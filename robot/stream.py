import threading
import time
from flask import Flask, Response
import cv2
import numpy as np

from robot.laser_tracker import LaserTracker


def create_stream_app(tracker: LaserTracker) -> Flask:
    """Create a Flask app that streams frames from a LaserTracker instance.
    
    Args:
        tracker: A LaserTracker instance to read frames from.
        
    Returns:
        A Flask app configured with MJPEG stream endpoints.
    """
    app = Flask(__name__)

    def generate_mjpeg_stream(view_name: str):
        """Generator function for MJPEG stream."""
        last_seq = -1
        while True:
            seq, _ = tracker.get_frame_metadata()
            if seq == last_seq:
                time.sleep(0.002)
                continue

            frames = tracker.get_current_frames(copy_frames=False)
            frame = frames.get(view_name)
            if frame is None:
                time.sleep(0.002)
                continue
            last_seq = seq

            ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ret:
                time.sleep(0.002)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(buffer)).encode() + b"\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )

    @app.route("/stream/raw")
    def stream_raw():
        return Response(
            generate_mjpeg_stream("raw"),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/mask")
    def stream_mask():
        return Response(
            generate_mjpeg_stream("mask"),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/detection")
    def stream_detection():
        return Response(
            generate_mjpeg_stream("detection"),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/hsv")
    def stream_hsv():
        return Response(
            generate_mjpeg_stream("hsv"),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def run_stream_server(tracker: LaserTracker, host: str = "0.0.0.0", port: int = 5000) -> None:
    """Run the Flask streaming server in the current thread.
    
    Args:
        tracker: A LaserTracker instance to read frames from.
        host: Host to bind to.
        port: Port to bind to.
    """
    app = create_stream_app(tracker)
    app.run(host=host, port=port, debug=False, threaded=True)