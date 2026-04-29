# Checkerboard Calibration Workflow

`calibrate_checkerboard_extrinsics.py` estimates the transform expected by `track_test_2.cpp`:

```text
p_base_depth = aux_translation_mm + rotation_matrix * p_aux_depth
```

In other words, it outputs the `aux depth -> base depth` transform that should be copied into:

- `aux_translation_mm`
- `rotation_matrix`

## Inputs

1. A directory of base-camera checkerboard images.
2. A directory of aux-camera checkerboard images.
3. A rig calibration JSON based on `checkerboard_rig_template.json`.

Image pairs are matched by stem:

```text
base/0001.png <-> aux/0001.png
base/0002.png <-> aux/0002.png
```

The rig calibration JSON must contain:

- each device's color camera matrix
- each device's color distortion coefficients
- either `color_to_depth_*` or `depth_to_color_*`

The script assumes:

- checkerboard detection is done in color images
- body-tracking / fusion coordinates live in each device's depth coordinate system

So it first solves the checkerboard pose in color, then converts that pose into depth coordinates before solving the final aux-to-base transform.

## Dependencies

Use any Python environment that has:

- `numpy`
- `opencv-python`

Example:

```bat
python -m pip install numpy opencv-python
```

## Example

```bat
python .\tools\calibrate_checkerboard_extrinsics.py ^
  --base-dir .\calibration_images\base ^
  --aux-dir .\calibration_images\aux ^
  --rig-calibration .\tools\checkerboard_rig_template.json ^
  --board-cols 9 ^
  --board-rows 6 ^
  --square-size-mm 25 ^
  --output-json .\calibration_images\checkerboard_result.json ^
  --config-in .\track_config_2.json ^
  --config-out .\track_config_2_checkerboard.json
```

## Outputs

The script prints:

- the estimated `aux_translation_mm`
- the estimated `rotation_matrix`
- mean reprojection errors
- per-pair translation / rotation diagnostics

If `--config-in` and `--config-out` are provided, it also writes a patched tracking config JSON that can be copied into normal runtime use.

## Practical Notes

- Use multiple checkerboard poses and distances, not a single frame.
- Keep the whole checkerboard visible in both images.
- Remove poor frames if reprojection error is obviously worse than the rest.
- The current repo still treats `aux_translation_mm` / `rotation_matrix` as calibration values under active refinement, so validate the result with real body-tracking logs after patching the config.
