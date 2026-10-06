"""Bounded dual Azure Kinect acquisition for calibration tools (Windows/x64)."""
from __future__ import annotations

from collections import deque
import ctypes as ct
from dataclasses import dataclass
import os
from pathlib import Path
import statistics
import threading
import time

import numpy as np

COLOR_RESOLUTIONS = {"720p": 1, "1080p": 2, "1440p": 3}


class DeviceConfiguration(ct.Structure):
    _fields_ = [
        ("color_format", ct.c_int), ("color_resolution", ct.c_int),
        ("depth_mode", ct.c_int), ("camera_fps", ct.c_int),
        ("synchronized_images_only", ct.c_bool),
        ("depth_delay_off_color_usec", ct.c_int32), ("wired_sync_mode", ct.c_int),
        ("subordinate_delay_off_master_usec", ct.c_uint32),
        ("disable_streaming_indicator", ct.c_bool),
    ]


@dataclass
class CameraFrame:
    sequence: int
    received_at: float
    color: np.ndarray  # original BGRA pixels; never rotate the stored image
    depth: np.ndarray  # uint16 millimeters
    color_timestamp_us: int
    depth_timestamp_us: int
    depth_system_timestamp_ns: int


@dataclass
class CapturePair:
    base: CameraFrame
    aux: CameraFrame
    expected_depth_delta_us: float

    @property
    def sync_error_us(self) -> float:
        return self.aux.depth_timestamp_us - self.base.depth_timestamp_us - self.expected_depth_delta_us


def choose_pair(base_frames, aux_frames, offset_us, tolerance_us=2000):
    """Choose the newest matching pair; a one-frame lag must not pass a phase check."""
    matches = [
        (base, aux) for base in base_frames for aux in aux_frames
        if abs(aux.depth_timestamp_us - base.depth_timestamp_us - offset_us) <= tolerance_us
    ]
    return max(matches, key=lambda p: min(p[0].received_at, p[1].received_at)) if matches else None


class DualKinectCapture:
    def __init__(self, dll_directory: Path, base_serial: str, aux_serial: str,
                 delay_us: int = 160, exposure_us: int = 30000, color_resolution: str = "720p"):
        if os.name != "nt" or ct.sizeof(ct.c_void_p) != 8:
            raise RuntimeError("Live Kinect capture requires 64-bit Python on Windows.")
        if not base_serial or not aux_serial or base_serial == aux_serial:
            raise ValueError("Set two different base/aux serial numbers.")
        if not 0 <= delay_us < 33333:
            raise ValueError("Subordinate delay must be in [0, 33333) microseconds.")
        if not isinstance(exposure_us, int) or isinstance(exposure_us, bool) or not 500 <= exposure_us <= 33330:
            raise ValueError("Manual exposure must be an integer from 500 to 33330 microseconds at 30 FPS.")
        if color_resolution not in COLOR_RESOLUTIONS:
            raise ValueError("Color resolution must be 720p, 1080p, or 1440p.")
        self.color_resolution = COLOR_RESOLUTIONS[color_resolution]
        self._dll_directory = os.add_dll_directory(str(dll_directory.resolve()))
        try:
            self.sdk = ct.CDLL(str(dll_directory.resolve() / "k4a.dll"))
        except BaseException:
            self._dll_directory.close()
            raise
        signatures = {
            "k4a_device_get_installed_count": ([], ct.c_uint32),
            "k4a_device_open": ([ct.c_uint32, ct.POINTER(ct.c_void_p)], ct.c_int),
            "k4a_device_close": ([ct.c_void_p], None),
            "k4a_device_get_serialnum": ([ct.c_void_p, ct.c_char_p, ct.POINTER(ct.c_size_t)], ct.c_int),
            "k4a_device_get_sync_jack": ([ct.c_void_p, ct.POINTER(ct.c_bool), ct.POINTER(ct.c_bool)], ct.c_int),
            "k4a_device_get_color_control": ([ct.c_void_p, ct.c_int, ct.POINTER(ct.c_int), ct.POINTER(ct.c_int32)], ct.c_int),
            "k4a_device_set_color_control": ([ct.c_void_p, ct.c_int, ct.c_int, ct.c_int32], ct.c_int),
            "k4a_device_start_cameras": ([ct.c_void_p, ct.POINTER(DeviceConfiguration)], ct.c_int),
            "k4a_device_stop_cameras": ([ct.c_void_p], None),
            "k4a_device_get_capture": ([ct.c_void_p, ct.POINTER(ct.c_void_p), ct.c_int32], ct.c_int),
            "k4a_capture_release": ([ct.c_void_p], None),
            "k4a_capture_get_color_image": ([ct.c_void_p], ct.c_void_p),
            "k4a_capture_get_depth_image": ([ct.c_void_p], ct.c_void_p),
            "k4a_image_get_buffer": ([ct.c_void_p], ct.POINTER(ct.c_uint8)),
            "k4a_image_get_width_pixels": ([ct.c_void_p], ct.c_int),
            "k4a_image_get_height_pixels": ([ct.c_void_p], ct.c_int),
            "k4a_image_get_stride_bytes": ([ct.c_void_p], ct.c_int),
            "k4a_image_get_device_timestamp_usec": ([ct.c_void_p], ct.c_uint64),
            "k4a_image_get_system_timestamp_nsec": ([ct.c_void_p], ct.c_uint64),
            "k4a_image_release": ([ct.c_void_p], None),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.sdk, name)
            function.argtypes, function.restype = args, result
        self.serials = {"base": base_serial, "aux": aux_serial}
        self.delay_us, self.exposure_us = delay_us, exposure_us
        self.devices = {}
        self.device_info = {}
        self._original_exposure = {}
        self._started = []
        self._threads = []
        self._stop = threading.Event()
        self._condition = threading.Condition()
        self._queues = {"base": deque(maxlen=8), "aux": deque(maxlen=8)}
        self._offset_samples = []
        self._offset_pairs = set()
        self.offset_us = None
        self.error = None
        self.started_at = 0.0
        self.cleanup_warnings = []

    def open(self):
        try:
            for index in range(self.sdk.k4a_device_get_installed_count()):
                handle = ct.c_void_p()
                if self.sdk.k4a_device_open(index, ct.byref(handle)):
                    continue
                retained = False
                try:
                    buffer, size = ct.create_string_buffer(256), ct.c_size_t(256)
                    self._check(self.sdk.k4a_device_get_serialnum(handle, buffer, ct.byref(size)), "Read serial")
                    serial = buffer.value.decode("ascii")
                    role = next((r for r, s in self.serials.items() if s == serial), None)
                    if role is None:
                        continue
                    jack_in, jack_out = ct.c_bool(), ct.c_bool()
                    self._check(self.sdk.k4a_device_get_sync_jack(handle, ct.byref(jack_in), ct.byref(jack_out)), "Read sync jacks")
                    self.devices[role] = handle
                    self.device_info[role] = dict(serial=serial, index=index, sync_in=jack_in.value, sync_out=jack_out.value)
                    retained = True
                finally:
                    if not retained:
                        self.sdk.k4a_device_close(handle)
            if set(self.devices) != {"base", "aux"}:
                raise RuntimeError("Cannot open both configured Kinects. Check USB connections and close other camera apps.")
            base, aux = self.device_info["base"], self.device_info["aux"]
            if base["sync_out"] and not base["sync_in"] and aux["sync_in"]:
                self.master = "base"
            elif aux["sync_out"] and not aux["sync_in"] and base["sync_in"]:
                self.master = "aux"
            else:
                raise RuntimeError("Connect one Kinect's Sync Out to the other Kinect's Sync In.")
            for role, info in self.device_info.items():
                info["wired_sync_mode"] = "master" if role == self.master else "subordinate"
            return self
        except BaseException:
            self.close()
            raise

    def start(self):
        try:
            for role, handle in self.devices.items():
                mode, value = ct.c_int(), ct.c_int32()
                self._check(self.sdk.k4a_device_get_color_control(handle, 0, ct.byref(mode), ct.byref(value)), "Read exposure")
                self._original_exposure[role] = (mode.value, value.value)
                self._check(self.sdk.k4a_device_set_color_control(handle, 0, 1, self.exposure_us), "Set matching manual exposure")
            subordinate = "aux" if self.master == "base" else "base"
            for role in (subordinate, self.master):
                mode = 1 if role == self.master else 2
                config = DeviceConfiguration(3, self.color_resolution, 3, 2, True, 0, mode, self.delay_us if mode == 2 else 0, False)
                self._check(self.sdk.k4a_device_start_cameras(self.devices[role], ct.byref(config)), f"Start {role}")
                self._started.append(role)
            actual_exposures = []
            for role, handle in self.devices.items():
                mode, value = ct.c_int(), ct.c_int32()
                self._check(self.sdk.k4a_device_get_color_control(handle, 0, ct.byref(mode), ct.byref(value)), "Verify exposure")
                self.device_info[role]["exposure_us"] = value.value
                actual_exposures.append(value.value)
                if mode.value != 1:
                    raise RuntimeError(f"{role}: manual exposure was not applied.")
            if len(set(actual_exposures)) != 1:
                raise RuntimeError("The two Kinect exposure times differ after SDK adjustment.")
            self.started_at = time.monotonic()
            for role in ("base", "aux"):
                worker = threading.Thread(target=self._acquire, args=(role,), name=f"kinect-{role}", daemon=True)
                self._threads.append(worker)
                worker.start()
            return self
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _check(result, label):
        if result:
            raise RuntimeError(f"{label} failed (SDK result {result}).")

    def _copy_image(self, image, color):
        width = self.sdk.k4a_image_get_width_pixels(image)
        height = self.sdk.k4a_image_get_height_pixels(image)
        stride = self.sdk.k4a_image_get_stride_bytes(image)
        if width <= 0 or height <= 0 or stride < width * (4 if color else 2):
            raise RuntimeError("Invalid Kinect image dimensions.")
        raw = np.ctypeslib.as_array(self.sdk.k4a_image_get_buffer(image), shape=(height * stride,)).reshape(height, stride)
        pixels = raw[:, :width * (4 if color else 2)].copy()
        return pixels.reshape(height, width, 4) if color else pixels.view("<u2").reshape(height, width)

    def _acquire(self, role):
        sequence, timeouts = 0, 0
        try:
            while not self._stop.is_set():
                capture = ct.c_void_p()
                rc = self.sdk.k4a_device_get_capture(self.devices[role], ct.byref(capture), 100)
                if rc == 2:
                    timeouts += 1
                    if timeouts >= 50:
                        raise RuntimeError(f"{role}: no captures for five seconds.")
                    continue
                self._check(rc, f"Capture {role}")
                timeouts = 0
                color, depth = None, None
                try:
                    color = self.sdk.k4a_capture_get_color_image(capture)
                    depth = self.sdk.k4a_capture_get_depth_image(capture)
                    if not color or not depth:
                        continue
                    sequence += 1
                    frame = CameraFrame(
                        sequence, time.monotonic(), self._copy_image(color, True), self._copy_image(depth, False),
                        int(self.sdk.k4a_image_get_device_timestamp_usec(color)),
                        int(self.sdk.k4a_image_get_device_timestamp_usec(depth)),
                        int(self.sdk.k4a_image_get_system_timestamp_nsec(depth)),
                    )
                    with self._condition:
                        self._queues[role].append(frame)
                        self._learn_offset(role, frame)
                        self._condition.notify_all()
                finally:
                    if color:
                        self.sdk.k4a_image_release(color)
                    if depth:
                        self.sdk.k4a_image_release(depth)
                    self.sdk.k4a_capture_release(capture)
        except BaseException as exc:
            with self._condition:
                self.error = exc
                self._stop.set()
                self._condition.notify_all()

    def _learn_offset(self, role, frame):
        # Device timestamps have an undefined absolute origin. Learn the full offset
        # from frames closest in host arrival time, then pair without frame-period modulo.
        if self.offset_us is not None or frame.received_at - self.started_at < 1.0:
            return
        other = "aux" if role == "base" else "base"
        if not self._queues[other]:
            return
        match = min(self._queues[other], key=lambda f: abs(f.depth_system_timestamp_ns - frame.depth_system_timestamp_ns))
        if abs(match.depth_system_timestamp_ns - frame.depth_system_timestamp_ns) > 15_000_000:
            return
        base, aux = (frame, match) if role == "base" else (match, frame)
        key = (base.sequence, aux.sequence)
        if key not in self._offset_pairs:
            self._offset_pairs.add(key)
            self._offset_samples.append(aux.depth_timestamp_us - base.depth_timestamp_us)
        if len(self._offset_samples) >= 20:
            offset = float(statistics.median(self._offset_samples))
            inliers = [s for s in self._offset_samples if abs(s - offset) <= 2000]
            if len(inliers) < 16:
                raise RuntimeError("Unstable wired synchronization during initial clock-offset measurement.")
            self.offset_us = float(statistics.median(inliers))

    def next_pair(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                if self.error:
                    raise RuntimeError(str(self.error)) from self.error
                if self._stop.is_set():
                    raise RuntimeError("Capture stopped.")
                if self.offset_us is not None:
                    now = time.monotonic()
                    base = [f for f in self._queues["base"] if now - f.received_at <= 0.5]
                    aux = [f for f in self._queues["aux"] if now - f.received_at <= 0.5]
                    matched = choose_pair(base, aux, self.offset_us)
                    if matched:
                        for role, frame in zip(("base", "aux"), matched):
                            while self._queues[role] and self._queues[role][0].sequence <= frame.sequence:
                                self._queues[role].popleft()
                        return CapturePair(*matched, self.offset_us)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Waiting for fresh, aligned captures from both Kinects.")
                self._condition.wait(min(remaining, 0.1))

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        for worker in self._threads:
            worker.join(timeout=2)
        for role in reversed(self._started):
            self.sdk.k4a_device_stop_cameras(self.devices[role])
        for role, handle in self.devices.items():
            if role in self._original_exposure:
                mode, value = self._original_exposure[role]
                if self.sdk.k4a_device_set_color_control(handle, 0, mode, value):
                    self.cleanup_warnings.append(f"Could not restore {role} exposure.")
            self.sdk.k4a_device_close(handle)
        self.devices.clear()
        self._started.clear()
        self._dll_directory.close()

    def __enter__(self):
        return self.open().start()

    def __exit__(self, *_):
        self.close()
