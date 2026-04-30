from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import threading
from pathlib import Path
import time

import cv2
import numpy as np

import json

try:
    from picamera2 import Picamera2
except ImportError:  # pragma: no cover - only used on the Pi runtime
    Picamera2 = None

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "config.json"

with open(CONFIG_PATH, "r") as f:
    config = json.load(f)

DEFAULT_EXPOSURE = config["tuning"]["exposure"]
DEFAULT_SATURATION_LOWER = config["tuning"].get("saturation_low", 80)
DEFAULT_SATURATION_UPPER = config["tuning"].get("saturation_high", 255)
DEFAULT_BRIGHTNESS_LOWER = config["tuning"]["brightness_low"]
DEFAULT_BRIGHTNESS_UPPER = config["tuning"]["brightness_high"]
DEFAULT_GREEN_H_LOWER = config["tuning"]["hue_low"]
DEFAULT_GREEN_H_UPPER = config["tuning"]["hue_high"]



@dataclass
class DotDetection:
    center: Tuple[int, int]
    area: float


@dataclass
class DotPosition:
    """Normalized dot position with both axes mapped to [-1, 1]."""

    x: float
    y: float


def to_normalized_coordinates(detection: DotDetection, frame: np.ndarray) -> DotPosition:
    """Convert pixel coordinates to normalized control-space coordinates."""

    height, width = frame.shape[:2]
    half_width = width / 2.0
    half_height = height / 2.0
    x = (detection.center[0] - half_width) / half_width
    y = (half_height - detection.center[1]) / half_height
    x = float(max(-1.0, min(1.0, x)))
    y = float(max(-1.0, min(1.0, y)))
    return DotPosition(x=x, y=y)


class LaserTracker:
    """Owns the camera and produces dot positions for the control loop."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        hue_low: int = DEFAULT_GREEN_H_LOWER,
        hue_high: int = DEFAULT_GREEN_H_UPPER,
        saturation_low: int = DEFAULT_SATURATION_LOWER,
        saturation_high: int = DEFAULT_SATURATION_UPPER,
        brightness_low: int = DEFAULT_BRIGHTNESS_LOWER,
        brightness_high: int = DEFAULT_BRIGHTNESS_UPPER,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.hue_low = hue_low
        self.hue_high = hue_high
        self.saturation_low = saturation_low
        self.saturation_high = saturation_high
        self.brightness_low = brightness_low
        self.brightness_high = brightness_high
        self.camera_error: Optional[str] = None
        try:
            self.camera = open_camera(width, height, fps)
        except Exception as exc:
            # Keep the app alive even if camera init fails; stream endpoints
            # will return empty frames until a restart succeeds.
            self.camera = None
            self.camera_error = str(exc)
        self.exposure = DEFAULT_EXPOSURE
        self._latest_position: Optional[DotPosition] = None
        self._position_lock = threading.Lock()
        self._current_frames = {
            "raw": None,
            "mask": None,
            "detection": None,
            "hsv": None,
        }
        self._frames_lock = threading.Lock()
        self._frame_seq = 0
        self._latest_frame_time = 0.0
        self._fps_window_count = 0
        self._fps_window_start = time.monotonic()
        self._processing_fps = 0.0

    def update_tuning(
        self,
        exposure: int | None = None,
        hue_low: int | None = None,
        hue_high: int | None = None,
        saturation_low: int | None = None,
        saturation_high: int | None = None,
        brightness_low: int | None = None,
        brightness_high: int | None = None,
    ) -> None:
        """Update tuning parameters at runtime."""
        if exposure is not None and exposure != self.exposure and self.camera is not None:
            self.exposure = exposure
            apply_exposure(self.camera, exposure)
        if hue_low is not None:
            self.hue_low = hue_low
        if hue_high is not None:
            self.hue_high = hue_high
        if saturation_low is not None:
            self.saturation_low = saturation_low
        if saturation_high is not None:
            self.saturation_high = saturation_high
        if brightness_low is not None:
            self.brightness_low = brightness_low
        if brightness_high is not None:
            self.brightness_high = brightness_high

    def get_current_frames(self, copy_frames: bool = True) -> dict:
        """Get current processed frames (raw, mask, detection, hsv)."""
        with self._frames_lock:
            if not copy_frames:
                return {
                    "raw": self._current_frames["raw"],
                    "mask": self._current_frames["mask"],
                    "detection": self._current_frames["detection"],
                    "hsv": self._current_frames["hsv"],
                }
            return {
                "raw": self._current_frames["raw"].copy() if self._current_frames["raw"] is not None else None,
                "mask": self._current_frames["mask"].copy() if self._current_frames["mask"] is not None else None,
                "detection": self._current_frames["detection"].copy() if self._current_frames["detection"] is not None else None,
                "hsv": self._current_frames["hsv"].copy() if self._current_frames["hsv"] is not None else None,
            }

    def get_frame_metadata(self) -> tuple[int, float]:
        with self._frames_lock:
            return self._frame_seq, self._latest_frame_time

    def get_processing_fps(self) -> float:
        return self._processing_fps

    def set_latest_position(self, position: Optional[DotPosition]) -> None:
        with self._position_lock:
            self._latest_position = position

    def get_latest_position(self) -> Optional[DotPosition]:
        with self._position_lock:
            return self._latest_position

    def capture_and_process_frame(self) -> Optional[DotPosition]:
        """Capture one frame, process it, store it, and return the dot position if detected."""
        if self.camera is None:
            self.set_latest_position(None)
            return None

        frame = read_frame(self.camera)
        if frame is None:
            return None

        hsv, mask = build_green_mask(
            frame,
            self.hue_low,
            self.hue_high,
            self.saturation_low,
            self.saturation_high,
            self.brightness_low,
            self.brightness_high,
        )
        detection = detect_green_dot(hsv, mask)

        detection_frame = frame.copy()
        if detection is not None:
            detection_frame = draw_detection(detection_frame, detection)
        else:
            cv2.putText(
                detection_frame,
                "No green dot detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        raw_frame = label_panel(frame, "Raw")
        mask_frame = label_panel(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), "Mask")
        detection_frame = label_panel(detection_frame, "Detection")
        hsv_frame = label_panel(make_hsv_visualization(hsv), "HSV View")

        with self._frames_lock:
            self._current_frames["raw"] = raw_frame
            self._current_frames["mask"] = mask_frame
            self._current_frames["detection"] = detection_frame
            self._current_frames["hsv"] = hsv_frame
            self._frame_seq += 1
            self._latest_frame_time = time.monotonic()

        self._fps_window_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._processing_fps = self._fps_window_count / elapsed
            self._fps_window_count = 0
            self._fps_window_start = now

        if detection is None:
            self.set_latest_position(None)
            return None

        position = to_normalized_coordinates(detection, frame)
        self.set_latest_position(position)
        return position

    def close(self) -> None:
        if self.camera is not None and hasattr(self.camera, "stop"):
            self.camera.stop()

    def __enter__(self) -> "LaserTracker":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


def build_green_mask(
    frame: np.ndarray,
    hue_low: int,
    hue_high: int,
    saturation_low: int,
    saturation_high: int,
    brightness_low: int,
    brightness_high: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return HSV image and binary mask for the current tuning values."""

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([hue_low, saturation_low, brightness_low], dtype=np.uint8)
    upper_green = np.array([hue_high, saturation_high, brightness_high], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    return hsv, mask


def detect_green_dot(hsv: np.ndarray, mask: np.ndarray) -> Optional[DotDetection]:
    """Return the centroid of the brightest allowed blob in the mask."""

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    value_channel = hsv[:, :, 2]
    best_contour = None
    best_area = 0.0
    best_score = (-1.0, -1.0, -1.0)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 5:
            continue

        contour_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        contour_values = value_channel[contour_mask > 0]
        if contour_values.size == 0:
            continue

        max_brightness = float(np.max(contour_values))
        p99_brightness = float(np.percentile(contour_values, 99))
        bright_pixel_count = float(np.count_nonzero(contour_values >= max_brightness - 2))
        score = (max_brightness, p99_brightness, bright_pixel_count)

        if score > best_score:
            best_score = score
            best_contour = contour
            best_area = area

    if best_contour is None:
        return None

    moments = cv2.moments(best_contour)
    if moments["m00"] == 0:
        return None

    center_x = int(moments["m10"] / moments["m00"])
    center_y = int(moments["m01"] / moments["m00"])
    return DotDetection(center=(center_x, center_y), area=best_area)


def draw_detection(frame: np.ndarray, detection: DotDetection) -> np.ndarray:
    annotated = frame.copy()
    x, y = detection.center
    cv2.circle(annotated, (x, y), 10, (0, 0, 255), 2)
    cv2.putText(
        annotated,
        f"({x}, {y}) area={detection.area:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return annotated


def label_panel(panel: np.ndarray, text: str) -> np.ndarray:
    labeled = panel.copy()
    cv2.rectangle(labeled, (0, 0), (labeled.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(
        labeled,
        text,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return labeled


def make_hsv_visualization(hsv: np.ndarray) -> np.ndarray:
    """Create a hue-focused visualization from the HSV frame."""

    hue = hsv[:, :, 0]
    hue_scaled = cv2.convertScaleAbs(hue, alpha=255.0 / 179.0)
    hue_color = cv2.applyColorMap(hue_scaled, cv2.COLORMAP_HSV)

    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    visibility_mask = cv2.bitwise_and(sat, val)
    visibility_mask = cv2.threshold(visibility_mask, 30, 255, cv2.THRESH_BINARY)[1]

    return cv2.bitwise_and(hue_color, hue_color, mask=visibility_mask)


def apply_exposure(camera: object, exposure_value: int) -> None:
    if exposure_value == 0:
        camera.set_controls({"AeEnable": True})
        return

    exposure_time = int(100 + (exposure_value / 100.0) * 32900)
    camera.set_controls({"AeEnable": False, "ExposureTime": exposure_time})


def open_camera(width: int, height: int, fps: int, retries: int = 5, retry_delay: float = 1.0) -> object:
    if Picamera2 is None:
        raise RuntimeError("picamera2 is not available in this environment")

    last_error: Optional[Exception] = None
    for attempt in range(1, retries + 1):
        camera = None
        try:
            camera = Picamera2()
            config = camera.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
            camera.configure(config)
            camera.start()
            camera.set_controls({"FrameRate": fps})
            apply_exposure(camera, DEFAULT_EXPOSURE)
            return camera
        except Exception as exc:
            last_error = exc
            if camera is not None and hasattr(camera, "close"):
                try:
                    camera.close()
                except Exception:
                    pass
            if attempt < retries:
                time.sleep(retry_delay * attempt)

    raise RuntimeError(
        f"Failed to initialize Picamera2 after {retries} attempts. Last error: {last_error}"
    )


def read_frame(camera: object) -> Optional[np.ndarray]:
    if camera is None:
        return None
    frame = camera.capture_array()
    if frame is None:
        return None
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
