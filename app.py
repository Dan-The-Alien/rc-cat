from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass
import streamlit as st
import socket

# Must import after streamlit
from robot.laser_tracker import LaserTracker
from robot.pid_controller import PIDConfig, PIDController
from robot.stream import run_stream_server
from robot.hardware_controller import ArduinoController

st.set_page_config(page_title="RC Cat Laser Tracker", layout="wide")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")

st.title("RC Cat Laser Tracker")
st.caption("Live MJPEG streams from the camera feed and green dot detection.")


def get_server_host() -> str:
    """Return an IP/hostname clients should use to reach this server.

    Order of preference:
    - STREAMLIT_SERVER_HOST env var (explicit override)
    - primary non-loopback IP detected by opening a UDP socket
    - fallback to localhost
    """
    host = os.environ.get("STREAMLIT_SERVER_HOST")
    if host:
        return host
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # use a public IP; no packets are actually sent
        s.connect(("8.8.8.8", 80))
        addr = s.getsockname()[0]
        s.close()
        return addr
    except Exception:
        return "localhost"


SERVER_HOST = get_server_host()


def load_config() -> dict:
    """Load configuration from config.json."""
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r") as f:
            config = json.load(f)
    else:
        config = {
            "camera": {"width": 640, "height": 480, "fps": 30},
            "tuning": {
                "exposure": 2,
                "hue_low": 31,
                "hue_high": 83,
                "saturation_low": 80,
                "saturation_high": 255,
                "brightness_low": 12,
                "brightness_high": 255,
            },
            "pid": {
                "x": {"kp": 0.6, "ki": 0.0, "kd": 0.05, "target": 0.0},
                "y": {"kp": 0.6, "ki": 0.0, "kd": 0.05, "target": 0.0},
            },
            "hardware": {
                "arduino_port": "/dev/ttyUSB0",
                "steering_center": 1500,
                "steering_range": 500,
                "throttle_idle": 1500,
                "throttle_max": 2000,
            },
        }

    config.setdefault("camera", {})
    config.setdefault("tuning", {})
    config.setdefault("pid", {})
    config.setdefault("hardware", {})
    config["pid"].setdefault("x", {})
    config["pid"].setdefault("y", {})
    config["camera"].setdefault("width", 640)
    config["camera"].setdefault("height", 480)
    config["camera"].setdefault("fps", 30)
    config["tuning"].setdefault("exposure", 2)
    config["tuning"].setdefault("hue_low", 31)
    config["tuning"].setdefault("hue_high", 83)
    config["tuning"].setdefault("saturation_low", 80)
    config["tuning"].setdefault("saturation_high", 255)
    config["tuning"].setdefault("brightness_low", 12)
    config["tuning"].setdefault("brightness_high", 255)
    config["pid"]["x"].setdefault("kp", 0.6)
    config["pid"]["x"].setdefault("ki", 0.0)
    config["pid"]["x"].setdefault("kd", 0.05)
    config["pid"]["x"].setdefault("target", 0.0)
    config["pid"]["y"].setdefault("kp", 0.6)
    config["pid"]["y"].setdefault("ki", 0.0)
    config["pid"]["y"].setdefault("kd", 0.05)
    config["pid"]["y"].setdefault("target", 0.0)
    config["hardware"].setdefault("arduino_port", "/dev/ttyUSB0")
    config["hardware"].setdefault("steering_center", 1500)
    config["hardware"].setdefault("steering_range", 500)
    config["hardware"].setdefault("throttle_idle", 1500)
    config["hardware"].setdefault("throttle_max", 2000)
    return config


def save_config(config: dict) -> None:
    """Save configuration to config.json."""
    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=2)


def capture_frames_loop(tracker: LaserTracker) -> None:
    """Continuously capture and process frames."""
    try:
        while True:
            tracker.capture_and_process_frame()
            time.sleep(0.001)  # Small delay to prevent CPU spinning
    except Exception as e:
        st.error(f"Error in frame capture: {e}")


@st.cache_resource
def get_shared_tracker() -> LaserTracker:
    """Create and cache a single tracker instance for the whole Streamlit process."""
    config = load_config()
    tracker = LaserTracker(
        width=config["camera"]["width"],
        height=config["camera"]["height"],
        fps=config["camera"]["fps"],
        hue_low=config["tuning"]["hue_low"],
        hue_high=config["tuning"]["hue_high"],
        saturation_low=config["tuning"]["saturation_low"],
        saturation_high=config["tuning"]["saturation_high"],
        brightness_low=config["tuning"]["brightness_low"],
        brightness_high=config["tuning"]["brightness_high"],
    )

    capture_thread = threading.Thread(
        target=capture_frames_loop,
        args=(tracker,),
        daemon=True,
    )
    capture_thread.start()

    flask_thread = threading.Thread(
        target=run_stream_server,
        args=(tracker,),
        daemon=True,
    )
    flask_thread.start()

    return tracker


def build_pid_config(axis_config: dict) -> PIDConfig:
    return PIDConfig(
        kp=float(axis_config["kp"]),
        ki=float(axis_config["ki"]),
        kd=float(axis_config["kd"]),
        target=float(axis_config["target"]),
    )


@dataclass
class ControlSnapshot:
    position_x: float = float("nan")
    position_y: float = float("nan")
    setpoint_x: float = 0.0
    setpoint_y: float = 0.0
    has_lock: bool = False
    throttle_enabled: bool = False
    arduino_connected: bool = False
    control_hz: float = 0.0
    processing_fps: float = 0.0


def control_loop(
    tracker: LaserTracker,
    pid_x: PIDController,
    pid_y: PIDController,
    arduino: ArduinoController,
    snapshot: ControlSnapshot,
    control_lock: threading.Lock,
) -> None:
    """Run PID and hardware output at a fixed rate independent of Streamlit reruns."""
    target_hz = 100.0
    tick_interval = 1.0 / target_hz
    last_update = time.monotonic()
    next_tick = last_update + tick_interval
    rate_window_start = last_update
    rate_window_count = 0
    last_lock_time = last_update

    while True:
        now = time.monotonic()
        sleep_for = next_tick - now
        if sleep_for > 0:
            time.sleep(sleep_for)

        tick_start = time.monotonic()
        dt = max(tick_start - last_update, 1e-3)
        last_update = tick_start
        next_tick += tick_interval
        if (tick_start - next_tick) > (tick_interval * 5):
            next_tick = tick_start + tick_interval

        latest_position = tracker.get_latest_position()
        steering_output = -pid_x.last_output
        throttle_output = (-pid_y.last_output / 2.0) + 0.5
        throttle_output = max(0.0, min(1.0, throttle_output))
        has_lock = latest_position is not None
        throttle_enabled = True

        with control_lock:
            throttle_enabled = snapshot.throttle_enabled
            if latest_position is not None:
                pid_result_x = pid_x.step(latest_position.x, dt)
                pid_result_y = pid_y.step(latest_position.y, dt)

                steering_output = -pid_result_x.output
                throttle_output = (-pid_result_y.output / 2.0) + 0.5
                throttle_output = max(0.0, min(1.0, throttle_output))

                # Update last lock time when we have a valid position
                last_lock_time = tick_start

                snapshot.position_x = latest_position.x
                snapshot.position_y = latest_position.y
                snapshot.setpoint_x = steering_output
                snapshot.setpoint_y = throttle_output
                snapshot.has_lock = True
            else:
                snapshot.position_x = float("nan")
                snapshot.position_y = float("nan")
                snapshot.setpoint_x = steering_output
                snapshot.setpoint_y = throttle_output
                snapshot.has_lock = False

            if not throttle_enabled:
                throttle_output = 0.0
                snapshot.setpoint_y = throttle_output

            rate_window_count += 1
            elapsed = tick_start - rate_window_start
            if elapsed >= 0.5:
                snapshot.control_hz = rate_window_count / elapsed
                snapshot.processing_fps = tracker.get_processing_fps()
                rate_window_count = 0
                rate_window_start = tick_start

        if arduino.connected:
            # Safety: if we have not had a lock for over 0.5s, force throttle to 0
            if (tick_start - last_lock_time) > 0.5:
                throttle_output = 0.0
                snapshot.setpoint_y = throttle_output
            if not throttle_enabled:
                throttle_output = 0.0
                snapshot.setpoint_y = throttle_output
            arduino.set_steering(steering_output)
            arduino.set_throttle(throttle_output)

        snapshot.arduino_connected = arduino.connected


# Initialize session state
if "tracker" not in st.session_state:
    config = load_config()
    st.session_state.config = config

    # Create or reuse one shared tracker per Streamlit process.
    st.session_state.tracker = get_shared_tracker()

    if st.session_state.tracker.camera is None:
        camera_error = st.session_state.tracker.camera_error or "Unknown camera initialization error"
        st.warning(f"Camera initialization failed. Running in degraded mode. Error: {camera_error}")

    st.success("✓ Tracker and Flask server initialized")
    time.sleep(1)  # Give threads time to start

if "pid_x" not in st.session_state:
    st.session_state.pid_x = PIDController(build_pid_config(st.session_state.config["pid"]["x"]))
    st.session_state.pid_y = PIDController(build_pid_config(st.session_state.config["pid"]["y"]))
    st.session_state.last_pid_update = time.monotonic()

if "control_lock" not in st.session_state:
    st.session_state.control_lock = threading.Lock()

if "control_snapshot" not in st.session_state:
    st.session_state.control_snapshot = ControlSnapshot()
    st.session_state.control_snapshot.throttle_enabled = False

if "arduino" not in st.session_state:
    arduino_config = st.session_state.config.get("hardware", {})
    st.session_state.arduino = ArduinoController(
        port=arduino_config.get("arduino_port", "/dev/ttyUSB0"),
        baud=115200,
    )
    # Try to connect, but don't fail if it's not available
    st.session_state.arduino.connect()

if "control_thread" not in st.session_state:
    st.session_state.control_thread = threading.Thread(
        target=control_loop,
        args=(
            st.session_state.tracker,
            st.session_state.pid_x,
            st.session_state.pid_y,
            st.session_state.arduino,
            st.session_state.control_snapshot,
            st.session_state.control_lock,
        ),
        daemon=True,
    )
    st.session_state.control_thread.start()


tracker = st.session_state.tracker
config = st.session_state.config
pid_x = st.session_state.pid_x
pid_y = st.session_state.pid_y

with st.session_state.control_lock:
    throttle_enabled = st.session_state.control_snapshot.throttle_enabled

# Sidebar controls
with st.sidebar:
    st.header("Camera")
    width = st.slider(
        "Width",
        320,
        1280,
        config["camera"]["width"],
        step=16,
        key="width_slider",
    )
    height = st.slider(
        "Height",
        240,
        960,
        config["camera"]["height"],
        step=16,
        key="height_slider",
    )
    fps = st.slider(
        "FPS",
        5,
        60,
        config["camera"]["fps"],
        key="fps_slider",
    )

    st.header("Tuning")
    exposure = st.slider(
        "Exposure",
        0,
        100,
        config["tuning"]["exposure"],
        key="exposure_slider",
    )
    hue_low = st.slider(
        "Green H low",
        0,
        179,
        config["tuning"]["hue_low"],
        key="hue_low_slider",
    )
    hue_high = st.slider(
        "Green H high",
        0,
        179,
        config["tuning"]["hue_high"],
        key="hue_high_slider",
    )
    saturation_low = st.slider(
        "Saturation low",
        0,
        255,
        config["tuning"].get("saturation_low", 80),
        key="saturation_low_slider",
    )
    saturation_high = st.slider(
        "Saturation high",
        0,
        255,
        config["tuning"].get("saturation_high", 255),
        key="saturation_high_slider",
    )
    brightness_low = st.slider(
        "Brightness low",
        0,
        255,
        config["tuning"]["brightness_low"],
        key="brightness_low_slider",
    )
    brightness_high = st.slider(
        "Brightness high",
        0,
        255,
        config["tuning"]["brightness_high"],
        key="brightness_high_slider",
    )

    st.divider()
    st.header("Throttle Safety")
    status_label = "Enabled" if throttle_enabled else "Disabled"
    st.caption(f"Throttle output is currently {status_label}.")
    enable_col, disable_col = st.columns(2)
    enable_pressed = enable_col.button("Enable throttle", use_container_width=True)
    disable_pressed = disable_col.button("Disable throttle", use_container_width=True)

    if enable_pressed or disable_pressed:
        desired_state = enable_pressed
        with st.session_state.control_lock:
            st.session_state.control_snapshot.throttle_enabled = desired_state
            throttle_enabled = desired_state
        if desired_state:
            st.success("Throttle output enabled.")
        else:
            st.warning("Throttle output disabled; ESC will be held at idle.")
            st.session_state.arduino.set_throttle(0.0)

    st.divider()
    st.header("PID X")
    pid_x_target = st.slider(
        "X target",
        -1.0,
        1.0,
        float(config["pid"]["x"]["target"]),
        step=0.01,
        key="pid_x_target_slider",
    )
    pid_x_kp = st.slider(
        "X Kp",
        0.0,
        5.0,
        float(config["pid"]["x"]["kp"]),
        step=0.01,
        key="pid_x_kp_slider",
    )
    pid_x_ki = st.slider(
        "X Ki",
        0.0,
        5.0,
        float(config["pid"]["x"]["ki"]),
        step=0.01,
        key="pid_x_ki_slider",
    )
    pid_x_kd = st.slider(
        "X Kd",
        0.0,
        5.0,
        float(config["pid"]["x"]["kd"]),
        step=0.01,
        key="pid_x_kd_slider",
    )

    st.header("PID Y")
    pid_y_target = st.slider(
        "Y target",
        -1.0,
        1.0,
        float(config["pid"]["y"]["target"]),
        step=0.01,
        key="pid_y_target_slider",
    )
    pid_y_kp = st.slider(
        "Y Kp",
        0.0,
        5.0,
        float(config["pid"]["y"]["kp"]),
        step=0.01,
        key="pid_y_kp_slider",
    )
    pid_y_ki = st.slider(
        "Y Ki",
        0.0,
        5.0,
        float(config["pid"]["y"]["ki"]),
        step=0.01,
        key="pid_y_ki_slider",
    )
    pid_y_kd = st.slider(
        "Y Kd",
        0.0,
        5.0,
        float(config["pid"]["y"]["kd"]),
        step=0.01,
        key="pid_y_kd_slider",
    )

    st.divider()
    st.header("Hardware Control")
    
    arduino_port = st.text_input(
        "Arduino Port",
        config["hardware"]["arduino_port"],
        key="arduino_port_input",
    )
    # If the Arduino port was changed in the UI, reconnect the Arduino controller
    if "arduino" in st.session_state:
        arduino = st.session_state.arduino
        try:
            current_port = getattr(arduino, "port", None)
        except Exception:
            current_port = None
        if current_port != arduino_port:
            with st.session_state.control_lock:
                try:
                    arduino.disconnect()
                except Exception:
                    pass
                arduino.port = arduino_port
                arduino.connect()
                if arduino.connected:
                    st.success(f"Arduino reconnected on {arduino_port}")
                else:
                    st.warning(f"Arduino reconnect failed on {arduino_port}")
    
    st.subheader("Steering")
    steering_center = st.slider(
        "Steering Center (µs)",
        1000,
        2000,
        config["hardware"]["steering_center"],
        step=10,
        key="steering_center_slider",
    )
    steering_range = st.slider(
        "Steering Range (µs)",
        100,
        1000,
        config["hardware"]["steering_range"],
        step=10,
        key="steering_range_slider",
    )
    
    st.subheader("Throttle")
    throttle_idle = st.slider(
        "Throttle Idle (µs)",
        1000,
        1600,
        config["hardware"]["throttle_idle"],
        step=10,
        key="throttle_idle_slider",
    )
    throttle_max = st.slider(
        "Throttle Max (µs)",
        1600,
        2000,
        config["hardware"]["throttle_max"],
        step=10,
        key="throttle_max_slider",
    )
    
    # Update hardware config
    arduino = st.session_state.arduino
    with st.session_state.control_lock:
        arduino.update_steering_config(steering_center, steering_range)
        arduino.update_throttle_config(throttle_idle, throttle_max)

    # Update tracker with new tuning values immediately
    tracker.update_tuning(
        exposure=exposure,
        hue_low=hue_low,
        hue_high=hue_high,
        saturation_low=saturation_low,
        saturation_high=saturation_high,
        brightness_low=brightness_low,
        brightness_high=brightness_high,
    )

    pid_x_config = PIDConfig(kp=pid_x_kp, ki=pid_x_ki, kd=pid_x_kd, target=pid_x_target)
    pid_y_config = PIDConfig(kp=pid_y_kp, ki=pid_y_ki, kd=pid_y_kd, target=pid_y_target)
    with st.session_state.control_lock:
        if st.session_state.get("pid_config_x") != pid_x_config:
            pid_x.configure(pid_x_config)
            st.session_state.pid_config_x = pid_x_config
        if st.session_state.get("pid_config_y") != pid_y_config:
            pid_y.configure(pid_y_config)
            st.session_state.pid_config_y = pid_y_config

    # Save settings
    new_config = {
        "camera": {"width": width, "height": height, "fps": fps},
        "tuning": {
            "exposure": exposure,
            "hue_low": hue_low,
            "hue_high": hue_high,
            "saturation_low": saturation_low,
            "saturation_high": saturation_high,
            "brightness_low": brightness_low,
            "brightness_high": brightness_high,
        },
        "pid": {
            "x": {"kp": pid_x_kp, "ki": pid_x_ki, "kd": pid_x_kd, "target": pid_x_target},
            "y": {"kp": pid_y_kp, "ki": pid_y_ki, "kd": pid_y_kd, "target": pid_y_target},
        },
        "hardware": {
            "arduino_port": arduino_port,
            "steering_center": steering_center,
            "steering_range": steering_range,
            "throttle_idle": throttle_idle,
            "throttle_max": throttle_max,
        },
    }

    if new_config != config:
        save_config(new_config)
        st.session_state.config = new_config
        st.success("✓ Settings saved")

    st.divider()
    st.header("ℹ️ Information")
    st.markdown(
        f"""
        The camera streams are being served by a Flask backend at `http://{SERVER_HOST}:5000`
        
        Tuning parameters update in real-time.
        """
    )

# Display the four camera streams in a 2x2 grid
st.header("Live Camera Feed")

col1, col2 = st.columns(2)

with col1:
    st.subheader("Raw Input")
    st.markdown(
        f"""
        <img src="http://{SERVER_HOST}:5000/stream/raw" width="100%" style="border: 2px solid #ddd; border-radius: 4px;">
        """,
        unsafe_allow_html=True,
    )

with col2:
    st.subheader("Mask")
    st.markdown(
        f"""
        <img src="http://{SERVER_HOST}:5000/stream/mask" width="100%" style="border: 2px solid #ddd; border-radius: 4px;">
        """,
        unsafe_allow_html=True,
    )

col3, col4 = st.columns(2)

with col3:
    st.subheader("Detection")
    st.markdown(
        f"""
        <img src="http://{SERVER_HOST}:5000/stream/detection" width="100%" style="border: 2px solid #ddd; border-radius: 4px;">
        """,
        unsafe_allow_html=True,
    )

with col4:
    st.subheader("HSV View")
    st.markdown(
        f"""
        <img src="http://{SERVER_HOST}:5000/stream/hsv" width="100%" style="border: 2px solid #ddd; border-radius: 4px;">
        """,
        unsafe_allow_html=True,
    )


@st.fragment(run_every=0.2)
def render_pid_dashboard() -> None:
    snapshot = st.session_state.control_snapshot
    with st.session_state.control_lock:
        position_x = snapshot.position_x
        position_y = snapshot.position_y
        setpoint_x = snapshot.setpoint_x
        setpoint_y = snapshot.setpoint_y
        has_lock = snapshot.has_lock
        arduino_connected = snapshot.arduino_connected
        control_hz = snapshot.control_hz
        processing_fps = snapshot.processing_fps

    st.divider()
    st.header("PID Control")

    pid_metrics = st.columns(7)
    pid_metrics[0].metric("X position", f"{position_x:.3f}" if has_lock else "No lock")
    pid_metrics[1].metric("X setpoint", f"{setpoint_x:.3f}")
    pid_metrics[2].metric("Y position", f"{position_y:.3f}" if has_lock else "No lock")
    pid_metrics[3].metric("Y setpoint", f"{setpoint_y:.3f}")
    arduino_status = "🟢 Connected" if arduino_connected else "🔴 Disconnected"
    pid_metrics[4].metric("Arduino", arduino_status)
    pid_metrics[5].metric("Control Loop", f"{control_hz:.1f} Hz")
    pid_metrics[6].metric("Vision Loop", f"{processing_fps:.1f} FPS")


render_pid_dashboard()

# Display metrics
st.divider()
st.header("Current Settings")
metric_cols = st.columns(4)
metric_cols[0].metric("Exposure", exposure)
metric_cols[1].metric("Hue range", f"{hue_low} - {hue_high}")
metric_cols[2].metric("Saturation range", f"{saturation_low} - {saturation_high}")
metric_cols[3].metric("Brightness range", f"{brightness_low} - {brightness_high}")