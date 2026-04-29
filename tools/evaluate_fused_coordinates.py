#!/usr/bin/env python3
"""Evaluate fused coordinate traces exported by track_test_2.cpp.

This script reads the fusion trace CSV emitted by track_test_2.cpp and reports:

1. Internal consistency metrics that do not require ground truth.
2. Optional absolute error metrics against either:
   - a static known target position, or
   - a reference CSV with timestamps and x/y/z coordinates.

Examples
--------
Internal consistency only:

    python .\tools\evaluate_fused_coordinates.py ^
      --trace-csv .\fusion_eval\fusion_trace.csv

Against a static known target:

    python .\tools\evaluate_fused_coordinates.py ^
      --trace-csv .\fusion_eval\fusion_trace.csv ^
      --static-target-mm 0 0 1500

Against a reference CSV:

    python .\tools\evaluate_fused_coordinates.py ^
      --trace-csv .\fusion_eval\fusion_trace.csv ^
      --reference-csv .\fusion_eval\reference.csv ^
      --reference-time-col timestamp_us ^
      --reference-x-col x_mm ^
      --reference-y-col y_mm ^
      --reference-z-col z_mm
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class TraceRow:
    frame_index: int
    base_depth_ts_us: int | None
    aux_depth_ts_us: int | None
    sync_phase_us: int | None
    sync_phase_error_us: int | None
    source: str
    base_conf: str
    aux_body_conf: str
    base_found: bool
    aux_body_found: bool
    aux_depth_found: bool
    base_xyz: tuple[float, float, float] | None
    aux_body_in_base_xyz: tuple[float, float, float] | None
    aux_depth_in_base_xyz: tuple[float, float, float] | None
    fused_xyz: tuple[float, float, float] | None
    fused_minus_base_z_mm: float | None
    aux_match_err_mm: float | None
    aux_reason: str


@dataclass
class ReferenceRow:
    timestamp_us: int
    xyz: tuple[float, float, float]


def _optional_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    return float(value)


def _optional_int(value: str) -> int | None:
    value = value.strip()
    if not value:
        return None
    return int(value)


def _optional_vec3(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(_optional_float(row[key]) for key in keys)
    if values[0] is None:
        return None
    assert values[1] is not None and values[2] is not None
    return (values[0], values[1], values[2])


def _load_trace_rows(path: Path) -> list[TraceRow]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[TraceRow] = []
        for raw in reader:
            rows.append(
                TraceRow(
                    frame_index=int(raw["frame_index"]),
                    base_depth_ts_us=_optional_int(raw["base_depth_ts_us"]),
                    aux_depth_ts_us=_optional_int(raw["aux_depth_ts_us"]),
                    sync_phase_us=_optional_int(raw["sync_phase_us"]),
                    sync_phase_error_us=_optional_int(raw["sync_phase_error_us"]),
                    source=raw["source"],
                    base_conf=raw["base_conf"],
                    aux_body_conf=raw["aux_body_conf"],
                    base_found=raw["base_found"] == "1",
                    aux_body_found=raw["aux_body_found"] == "1",
                    aux_depth_found=raw["aux_depth_found"] == "1",
                    base_xyz=_optional_vec3(raw, ("base_x_mm", "base_y_mm", "base_z_mm")),
                    aux_body_in_base_xyz=_optional_vec3(
                        raw,
                        ("aux_body_in_base_x_mm", "aux_body_in_base_y_mm", "aux_body_in_base_z_mm"),
                    ),
                    aux_depth_in_base_xyz=_optional_vec3(
                        raw,
                        ("aux_depth_in_base_x_mm", "aux_depth_in_base_y_mm", "aux_depth_in_base_z_mm"),
                    ),
                    fused_xyz=_optional_vec3(raw, ("fused_x_mm", "fused_y_mm", "fused_z_mm")),
                    fused_minus_base_z_mm=_optional_float(raw["fused_minus_base_z_mm"]),
                    aux_match_err_mm=_optional_float(raw["aux_match_err_mm"]),
                    aux_reason=raw["aux_reason"],
                )
            )
    return rows


def _load_reference_rows(
    path: Path,
    time_col: str,
    x_col: str,
    y_col: str,
    z_col: str,
) -> list[ReferenceRow]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[ReferenceRow] = []
        for raw in reader:
            rows.append(
                ReferenceRow(
                    timestamp_us=int(raw[time_col]),
                    xyz=(float(raw[x_col]), float(raw[y_col]), float(raw[z_col])),
                )
            )
    rows.sort(key=lambda row: row.timestamp_us)
    return rows


def _pctl(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * q
    low = math.floor(pos)
    high = math.ceil(pos)
    if low == high:
        return sorted_values[low]
    weight = pos - low
    return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight


def _numeric_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": ordered[0],
        "p95": _pctl(ordered, 0.95),
        "max": ordered[-1],
    }


def _euclidean(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _count_by_source(rows: list[TraceRow]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        key = row.source or "(empty)"
        counts[key] = counts.get(key, 0) + 1
    return counts


def _trace_timestamp(row: TraceRow, column: str) -> int | None:
    if column == "base_depth_ts_us":
        return row.base_depth_ts_us
    if column == "aux_depth_ts_us":
        return row.aux_depth_ts_us
    raise ValueError(
        f"Unsupported --trace-time-col: {column}. "
        "Use base_depth_ts_us or aux_depth_ts_us."
    )


def _match_reference(
    trace_rows: list[TraceRow],
    reference_rows: list[ReferenceRow],
    trace_time_col: str,
    tolerance_us: int,
) -> list[tuple[TraceRow, ReferenceRow]]:
    timestamps = [row.timestamp_us for row in reference_rows]
    matches: list[tuple[TraceRow, ReferenceRow]] = []
    for trace_row in trace_rows:
        trace_timestamp = _trace_timestamp(trace_row, trace_time_col)
        if trace_timestamp is None or trace_row.fused_xyz is None:
            continue
        index = bisect_left(timestamps, trace_timestamp)
        candidates: list[ReferenceRow] = []
        if index < len(reference_rows):
            candidates.append(reference_rows[index])
        if index > 0:
            candidates.append(reference_rows[index - 1])
        if not candidates:
            continue
        best = min(candidates, key=lambda row: abs(row.timestamp_us - trace_timestamp))
        if abs(best.timestamp_us - trace_timestamp) <= tolerance_us:
            matches.append((trace_row, best))
    return matches


def _error_summary_from_pairs(
    pairs: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
) -> dict[str, Any] | None:
    if not pairs:
        return None

    axis_errors = {
        "x_mm": [pred[0] - ref[0] for pred, ref in pairs],
        "y_mm": [pred[1] - ref[1] for pred, ref in pairs],
        "z_mm": [pred[2] - ref[2] for pred, ref in pairs],
    }
    abs_axis_errors = {key: [abs(value) for value in values] for key, values in axis_errors.items()}
    euclidean_errors = [_euclidean(pred, ref) for pred, ref in pairs]

    return {
        "count": len(pairs),
        "axis_bias_mm": {key: statistics.fmean(values) for key, values in axis_errors.items()},
        "axis_mae_mm": {key: statistics.fmean(values) for key, values in abs_axis_errors.items()},
        "axis_rmse_mm": {
            key: math.sqrt(statistics.fmean([value * value for value in values]))
            for key, values in axis_errors.items()
        },
        "euclidean_mae_mm": statistics.fmean(euclidean_errors),
        "euclidean_rmse_mm": math.sqrt(statistics.fmean([value * value for value in euclidean_errors])),
        "euclidean_p95_mm": _pctl(sorted(euclidean_errors), 0.95),
        "euclidean_max_mm": max(euclidean_errors),
    }


def _pairwise_alignment_summary(
    rows: list[TraceRow],
    lhs_name: str,
    rhs_name: str,
    lhs_getter: Any,
    rhs_getter: Any,
) -> dict[str, Any]:
    euclidean_errors: list[float] = []
    z_deltas: list[float] = []

    for row in rows:
        lhs_xyz = lhs_getter(row)
        rhs_xyz = rhs_getter(row)
        if lhs_xyz is None or rhs_xyz is None:
            continue
        euclidean_errors.append(_euclidean(lhs_xyz, rhs_xyz))
        z_deltas.append(rhs_xyz[2] - lhs_xyz[2])

    return {
        "pair": f"{lhs_name}_vs_{rhs_name}",
        "count": len(euclidean_errors),
        "euclidean_mm": _numeric_summary(euclidean_errors),
        "rhs_minus_lhs_z_mm": _numeric_summary(z_deltas),
    }


def _internal_consistency_summary(rows: list[TraceRow]) -> dict[str, Any]:
    fused_rows = [row for row in rows if row.fused_xyz is not None]
    z_deltas = [row.fused_minus_base_z_mm for row in fused_rows if row.fused_minus_base_z_mm is not None]
    aux_match_errors = [row.aux_match_err_mm for row in rows if row.aux_match_err_mm is not None]

    fused_steps: list[float] = []
    last_fused: tuple[float, float, float] | None = None
    for row in fused_rows:
        if last_fused is not None:
            fused_steps.append(_euclidean(row.fused_xyz, last_fused))
        last_fused = row.fused_xyz

    source_groups: dict[str, list[TraceRow]] = {}
    for row in fused_rows:
        source_groups.setdefault(row.source or "(empty)", []).append(row)

    by_source: dict[str, Any] = {}
    for source, group in source_groups.items():
        by_source[source] = {
            "count": len(group),
            "z_delta_mm": _numeric_summary(
                [row.fused_minus_base_z_mm for row in group if row.fused_minus_base_z_mm is not None]
            ),
        }

    return {
        "total_rows": len(rows),
        "fused_rows": len(fused_rows),
        "fused_ratio": len(fused_rows) / len(rows) if rows else 0.0,
        "base_found_ratio": sum(row.base_found for row in rows) / len(rows) if rows else 0.0,
        "aux_body_found_ratio": sum(row.aux_body_found for row in rows) / len(rows) if rows else 0.0,
        "aux_depth_found_ratio": sum(row.aux_depth_found for row in rows) / len(rows) if rows else 0.0,
        "source_counts": _count_by_source(fused_rows),
        "z_delta_mm": _numeric_summary(z_deltas),
        "aux_match_err_mm": _numeric_summary(aux_match_errors),
        "fused_step_mm": _numeric_summary(fused_steps),
        "cross_camera_alignment": {
            "base_vs_aux_body": _pairwise_alignment_summary(
                rows,
                "base",
                "aux_body",
                lambda row: row.base_xyz,
                lambda row: row.aux_body_in_base_xyz,
            ),
            "base_vs_aux_depth": _pairwise_alignment_summary(
                rows,
                "base",
                "aux_depth",
                lambda row: row.base_xyz,
                lambda row: row.aux_depth_in_base_xyz,
            ),
            "aux_body_vs_aux_depth": _pairwise_alignment_summary(
                rows,
                "aux_body",
                "aux_depth",
                lambda row: row.aux_body_in_base_xyz,
                lambda row: row.aux_depth_in_base_xyz,
            ),
        },
        "by_source": by_source,
    }


def _print_summary(title: str, payload: dict[str, Any], indent: str = "") -> None:
    if title:
        print(title)
    for key, value in payload.items():
        if isinstance(value, dict):
            print(f"{indent}{key}:")
            _print_summary("", value, indent + "  ")
        else:
            if isinstance(value, float):
                if key.endswith("_ratio"):
                    rendered = f"{value:.3f}"
                else:
                    rendered = f"{value:.3f}"
            else:
                rendered = str(value)
            if title:
                print(f"{indent}- {key}: {rendered}")
            else:
                print(f"{indent}- {key}: {rendered}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate fused coordinate traces.")
    parser.add_argument("--trace-csv", required=True, type=Path, help="Fusion trace CSV exported by track_test_2.cpp.")
    accuracy_group = parser.add_mutually_exclusive_group()
    accuracy_group.add_argument("--static-target-mm", nargs=3, type=float, metavar=("X", "Y", "Z"))
    accuracy_group.add_argument("--reference-csv", type=Path, help="Optional reference CSV with timestamps and x/y/z columns.")
    parser.add_argument("--trace-time-col", default="base_depth_ts_us", help="Timestamp column to align against the reference CSV.")
    parser.add_argument("--reference-time-col", default="timestamp_us")
    parser.add_argument("--reference-x-col", default="x_mm")
    parser.add_argument("--reference-y-col", default="y_mm")
    parser.add_argument("--reference-z-col", default="z_mm")
    parser.add_argument("--match-tolerance-us", type=int, default=20000)
    parser.add_argument("--json-out", type=Path, help="Optional path to write the evaluation summary as JSON.")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    trace_rows = _load_trace_rows(args.trace_csv)
    consistency = _internal_consistency_summary(trace_rows)

    print("Fusion Trace Evaluation")
    print(f"- trace_csv: {args.trace_csv}")
    print(f"- rows: {len(trace_rows)}")
    print()
    _print_summary("Internal Consistency", consistency)

    evaluation_payload: dict[str, Any] = {
        "trace_csv": str(args.trace_csv),
        "internal_consistency": consistency,
    }

    fused_rows = [row for row in trace_rows if row.fused_xyz is not None]

    if args.static_target_mm is not None:
        target = tuple(args.static_target_mm)
        pairs = [(row.fused_xyz, target) for row in fused_rows]
        static_summary = _error_summary_from_pairs(pairs)
        evaluation_payload["static_target_mm"] = list(target)
        evaluation_payload["static_target_error"] = static_summary
        print()
        _print_summary("Static Target Error", static_summary or {"count": 0})
    elif args.reference_csv is not None:
        reference_rows = _load_reference_rows(
            args.reference_csv,
            args.reference_time_col,
            args.reference_x_col,
            args.reference_y_col,
            args.reference_z_col,
        )
        matches = _match_reference(
            trace_rows,
            reference_rows,
            args.trace_time_col,
            args.match_tolerance_us,
        )
        reference_summary = _error_summary_from_pairs([(trace.fused_xyz, ref.xyz) for trace, ref in matches])
        evaluation_payload["reference_csv"] = str(args.reference_csv)
        evaluation_payload["trace_time_col"] = args.trace_time_col
        evaluation_payload["reference_match_count"] = len(matches)
        evaluation_payload["reference_error"] = reference_summary
        print()
        _print_summary(
            "Reference Error",
            {
                "trace_time_col": args.trace_time_col,
                "match_count": len(matches),
                **(reference_summary or {}),
            },
        )
    else:
        print()
        print("Absolute error was not evaluated because no static target or reference CSV was provided.")
        print("Use --static-target-mm or --reference-csv when you need true accuracy metrics.")

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with args.json_out.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(evaluation_payload, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        print()
        print(f"Wrote JSON summary: {args.json_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
