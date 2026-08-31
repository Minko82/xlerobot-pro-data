#!/usr/bin/env python3
"""Live view of exactly what the recorder will capture.

The existing ``camera_stream.py`` opens the RealSense as a raw V4L2 device, which
bypasses the RealSense colour pipeline and green-casts the image. That is fine for
pointing the camera, but it is NOT what lands in the dataset -- ``record_kinesthetic.py``
goes through the RealSense SDK with ``ColorMode.RGB``. Judging framing and exposure
from the V4L2 view means judging a different image to the one the policy trains on.

So this opens the camera the same way the recorder does: same serial, same
resolution, same fps, same colour mode, same class. What you see here is what gets
written to disk, frame for frame.

    python scripts/camera_preview.py           # then open http://<robot-ip>:8080

STOP IT BEFORE RECORDING. The camera can only be opened by one process at a time,
so leaving this running makes record_kinesthetic.py fail to grab the device.

An optional grid overlay helps keep the workspace framed consistently across a
session -- a shifted view mid-dataset is the one inconsistency that quietly ruins
a recording, and with the neck bus disconnected the usual drift guard is off.

``--align`` is for putting the rig back where a dataset was recorded. The tape
bounding the workspace sat at x = 602.3 px in all twenty start frames of
glassbottle_pick_v3, sd 0.2 px, which makes it a far better reference than the
grid. The overlay draws that target and tracks the tape live, reporting the drift
in pixels and in the units that matter -- degrees, and normalised shoulder_pan.
Move the table until it reads zero. Run it with the arm AT THE START POSE: the
camera is wrist-mounted, so the target column only means anything from there.
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

PAGE = b"""<!doctype html><html><head><title>what the recorder sees</title>
<style>
 body{background:#0f1418;color:#e4eaef;font-family:system-ui,sans-serif;margin:0;padding:16px}
 h1{font-size:15px;font-weight:600;margin:0 0 4px}
 p{font-size:13px;color:#94a2ae;margin:0 0 12px}
 img{max-width:100%;border:1px solid #273037}
</style></head><body>
<h1>What the recorder sees</h1>
<p>RealSense SDK, RGB &mdash; identical to the frames record_kinesthetic.py writes.</p>
<img src="/stream.mjpg">
</body></html>"""

#: Tape position across the 20 start frames of glassbottle_pick_v3 (sd 0.2 px).
TAPE_X = 602.3

#: Geometry, to report drift in units that can be acted on.
DEG_PER_PX = 69.0 / 640                    # D435 colour horizontal FOV over frame width
DEG_PER_PAN_UNIT = ((3660 - 948) / 200.0) * (360.0 / 4096)   # from the live calibration

_latest: dict[str, bytes] = {}
_lock = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):          # keep the console readable
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    buf = _latest.get("jpg")
                if buf:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(buf)).encode() + b"\r\n\r\n")
                    self.wfile.write(buf + b"\r\n")
                time.sleep(0.04)
        except (BrokenPipeError, ConnectionResetError):
            pass


def find_tape(frame) -> float | None:
    """Brightness-weighted centroid of the rightmost bright vertical band.

    Returns None when the band touches the frame edge: a clipped band's centroid
    is pulled inward, so it would read as *less* drift than there really is.
    Saying nothing beats reporting a number that flatters the alignment.
    """
    v = frame[0:300, :].mean(axis=(0, 2)).astype(float)
    base = np.percentile(v, 50)
    thr = base + 0.55 * (v.max() - base)
    xs = np.arange(len(v))[v > thr]
    if len(xs) == 0:
        return None
    run = np.split(xs, np.where(np.diff(xs) > 3)[0] + 1)[-1]
    if len(run) < 3 or run[-1] >= frame.shape[1] - 2:
        return None
    w = v[run] - thr
    return float((run * w).sum() / w.sum())


def overlay(frame, grid: bool, align: bool):
    """Thirds grid and centre cross, so framing can be repeated between sessions."""
    f = frame.copy()
    h, w = f.shape[:2]
    if grid:
        for x in (w // 3, 2 * w // 3):
            cv2.line(f, (x, 0), (x, h), (60, 60, 60), 1)
        for y in (h // 3, 2 * h // 3):
            cv2.line(f, (0, y), (w, y), (60, 60, 60), 1)
        cv2.drawMarker(f, (w // 2, h // 2), (0, 200, 255), cv2.MARKER_CROSS, 18, 1)
    if align:
        cv2.line(f, (int(TAPE_X), 0), (int(TAPE_X), h), (0, 255, 0), 2)
        cv2.putText(f, "x=602 target", (int(TAPE_X) - 150, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        x = find_tape(frame)
        if x is None:
            cv2.putText(f, "TAPE OFF FRAME - bring it back into view", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            d = x - TAPE_X
            cv2.line(f, (int(x), 0), (int(x), h), (0, 0, 255), 1)
            good = abs(d) <= 3.0
            cv2.putText(f, "tape %+.1f px  %+.2f deg  %+.2f pan %s"
                        % (d, d * DEG_PER_PX, d * DEG_PER_PX / DEG_PER_PAN_UNIT,
                           "ALIGNED" if good else ""),
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 220, 0) if good else (0, 0, 255), 2)
    return f


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--camera-serial", default="838212073725")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--no-grid", action="store_true", help="Hide the framing grid.")
    p.add_argument("--align", action="store_true",
                   help="Track the workspace tape against the position it held while "
                        "glassbottle_pick_v3 was recorded. Arm must be at the start pose.")
    args = p.parse_args()

    cam = RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name=args.camera_serial, fps=args.fps,
        width=args.width, height=args.height, color_mode=ColorMode.RGB))
    print(f"\n  opening RealSense {args.camera_serial} "
          f"({args.width}x{args.height} @ {args.fps}) -- same path as the recorder")
    cam.connect()

    try:
        ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        ip = "127.0.0.1"
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"\n  live view:  http://{ip}:{args.port}")
    print(f"              http://xle-desktop.local:{args.port}")
    print("\n  STOP THIS (Ctrl-C) BEFORE RECORDING -- the camera opens once only.\n")

    n, t0 = 0, time.monotonic()
    try:
        while True:
            frame = cam.async_read()            # RGB, exactly as the dataset stores it
            bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)   # cv2 encodes BGR
            ok, jpg = cv2.imencode(".jpg", overlay(bgr, not args.no_grid, args.align),
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                with _lock:
                    _latest["jpg"] = jpg.tobytes()
            n += 1
            if n % 60 == 0:
                el = time.monotonic() - t0
                sys.stdout.write(f"\r    streaming {n} frames, {n/el:.1f} fps   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n  stopped -- camera released, safe to record now")
    finally:
        srv.shutdown()
        try:
            cam.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
