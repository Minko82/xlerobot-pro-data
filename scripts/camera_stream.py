#!/usr/bin/env python3
"""Serve the camera as a live MJPEG stream you can open in a browser.

Aiming a camera by capturing one frame at a time is slow and blind. This streams
it, so you can nudge the neck and watch the view move.

    python scripts/camera_stream.py            # then open http://<robot-ip>:8080

Stop it with Ctrl-C BEFORE recording -- a V4L2 device can only be opened once, so
leaving this running makes the recorder fail to grab the camera.
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = b"""<!doctype html><html><head><title>XLeRobot camera</title>
<style>body{background:#111;color:#ddd;font-family:system-ui;margin:0;padding:12px}
img{max-width:100%;border:1px solid #444}</style></head>
<body><p>Live view &mdash; /dev/videoIDX. Refresh if it stalls.</p>
<img src="/stream"></body></html>"""


class Cam:
    """One reader thread; every HTTP client is served the latest frame."""

    def __init__(self, index: int, width: int, height: int):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open /dev/video{index} -- is something else using it?")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame: bytes | None = None
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.stop.is_set():
            ok, fr = self.cap.read()
            if ok:
                enc, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if enc:
                    self.frame = buf.tobytes()
            else:
                time.sleep(0.05)

    def close(self):
        self.stop.set()
        time.sleep(0.2)
        self.cap.release()


CAM: Cam | None = None
INDEX = 4


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # the default logger prints a line per frame

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    f = CAM.frame if CAM else None
                    if f:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(f)).encode() +
                                         b"\r\n\r\n" + f + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE.replace(b"IDX", str(INDEX).encode()))


def main() -> int:
    global CAM, INDEX
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--camera-index", type=int, default=4)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--port", type=int, default=8080)
    args = p.parse_args()
    INDEX = args.camera_index

    CAM = Cam(args.camera_index, args.width, args.height)
    ip = socket.gethostbyname(socket.gethostname())
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"\n  Live view:  http://{ip}:{args.port}")
    print(f"              http://10.0.0.197:{args.port}   (if the above is a loopback address)")
    print("\n  Ctrl-C to stop. STOP THIS BEFORE RECORDING -- the camera can only be "
          "opened by one process.\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  stopping")
    finally:
        srv.shutdown()
        CAM.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
