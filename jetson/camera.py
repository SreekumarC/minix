"""Camera capture thread.

Grabs continuously so the web stream never waits on the sensor, runs
LDW on each frame, and keeps one encoded JPEG ready for any number of
browser clients.

Object detection is a second, mutually-exclusive processing mode on
the SAME capture loop and SAME /video output — no second stream, no
second route. Toggling it on runs YOLO instead of LDW per frame;
toggling it off goes back to normal. The YOLO model/ultralytics import
is lazy (only happens the first time detection is enabled) specifically
so that if it's missing or version-conflicts with this venv's
numpy/opencv, it fails ONLY when someone opts into detection mode,
not at Camera() construction — normal driving/AEB/LDW must keep working
even if detection never loads at all.
"""

import logging
import threading
import time
import config
import cv2

logger = logging.getLogger(__name__)

DEFAULT_ENGINE_PATH = "/home/sreekumar/minix-detect/yolo11n.engine"  # adjust if it lives elsewhere


def _pipeline(device, w, h, fps):
    return (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={w},height={h},framerate={fps}/1 ! "
        "jpegdec ! videoconvert ! "
        "appsink drop=1 max-buffers=1 sync=false"
    )


class Camera:
    def __init__(self, device, width, height, fps, quality, ldw=None,
                 detect_engine_path=DEFAULT_ENGINE_PATH):
        self.ldw = ldw
        self.quality = quality
        self.fps_measured = 0.0
        self.detect_engine_path = detect_engine_path

        self._detect_mode = False
        self._detect_model = None
        self._detect_load_error = None

        self.cap = cv2.VideoCapture(
            _pipeline(device, width, height, fps), cv2.CAP_GSTREAMER
        )
        
        if not self.cap.isOpened():
            # fall back to plain V4L2 if the GStreamer build is missing
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.ok = self.cap.isOpened()
        self.last_frame_ts = 0.0
        # Image quality for ACC's camera-health check (adas/acc.py):
        # bright = mean grey, contrast = grey std-dev, sharp = Laplacian var.
        # (Not self.quality — that is the JPEG quality used by imencode.)
        self.image_quality = {}
        self._frame_n = 0
        self._jpeg = None
        self._lock = threading.Lock()
        self._alive = True

        threading.Thread(target=self._loop, daemon=True).start()

    # ---------- detection mode ----------
    def set_detect_mode(self, enabled):
        """Returns (ok, error). error is None on success, or a message
        the caller can surface to the UI if enabling failed (e.g. the
        engine file is missing or ultralytics isn't installed in this
        venv — a real possibility, since the working YOLO setup lives
        in a SEPARATE venv with different pinned torch/numpy versions
        specifically to avoid conflicting with this one)."""
        if not enabled:
            self._detect_mode = False
            return True, None

        if self._detect_model is None:
            try:
                from ultralytics import YOLO  # lazy: only imported here
                self._detect_model = YOLO(self.detect_engine_path, task="detect")
            except Exception as e:
                self._detect_load_error = str(e)
                logger.warning("Detection model failed to load: %s", e)
                return False, self._detect_load_error

        self._detect_mode = True
        return True, None

    @property
    def detect_mode(self):
        return self._detect_mode

    def _loop(self):
        last = time.time()
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]

        while self._alive:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            if config.CAM_ROTATE_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            self.last_frame_ts = time.time()
            self._frame_n += 1
            if self._frame_n % 3 == 0:   # ~10 Hz is plenty, keep it cheap
                self.image_quality = self._measure_quality(frame)

            if self._detect_mode and self._detect_model is not None:
                try:
                    results = self._detect_model(frame, imgsz=640, conf=0.35, verbose=False)
                    frame = results[0].plot()
                except Exception as e:
                    # Detection failing mid-stream must not kill the whole
                    # camera thread — log once per failure, show the raw
                    # frame instead, and keep the loop alive.
                    logger.warning("Detection inference failed: %s", e)
            elif self.ldw is not None:
                frame = self.ldw.process(frame)

            ok, buf = cv2.imencode(".jpg", frame, params)
            if ok:
                with self._lock:
                    self._jpeg = buf.tobytes()

            now = time.time()
            dt = now - last
            last = now
            if dt > 0:
                self.fps_measured = 0.9 * self.fps_measured + 0.1 / dt

    @staticmethod
    def _measure_quality(frame):
        small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        mean, std = cv2.meanStdDev(gray)
        sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
        return {"bright": float(mean[0][0]), "contrast": float(std[0][0]),
                "sharp": float(sharp)}

    def frame(self):
        with self._lock:
            return self._jpeg

    def mjpeg(self):
        """Generator for a multipart MJPEG HTTP response."""
        boundary = b"--frame\r\n"
        while True:
            jpg = self.frame()
            if jpg is None:
                time.sleep(0.05)
                continue
            yield (boundary +
                   b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.03)

    def close(self):
        self._alive = False
        time.sleep(0.2)
        self.cap.release()
