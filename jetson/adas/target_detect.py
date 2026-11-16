"""Camera detection of the ACC target car (classical CV, no NN).

The target is a black mecanum RC car on a light floor, so it shows up as
a compact dark region low in the image. Each frame:

  1. downscale, take the floor region (lower part of the image)
  2. dark mask (low V, low saturation = black, not coloured), cleaned up
     with morphology so the floor's small grey speckles don't count
  3. candidate boxes filtered by size and shape (a car seen from behind
     is wider than tall; chair legs/cables are thin and get rejected)
  4. if a target has been LEARNED, each candidate is scored against the
     learned grey-level histogram (black body + white trim pattern);
     otherwise by how dark/solid it is
  5. temporal filter: "seen" only after hits in 3 of the last 5 frames

Learning ("Learn target" in the web UI): put the car straight ahead of
MiniX, 40-150 cm away, and press the button. The best candidate near the
image centre is stored with its histogram, darkness threshold, and
width x distance (from the ultrasonic) so later frames can estimate the
distance from box width alone (pinhole: distance ~ k / width_px). The
model is saved to target_model.json next to app.py and reloaded at boot.

Fail-safe like the rest: any exception in process() is logged once and
the detector reports nothing for that frame.
"""

import collections
import json
import logging
import os

import cv2
import numpy as np

logger = logging.getLogger(__name__)

WORK_W = 320                 # processing width (keeps it ~1-2 ms per frame)
ROI_TOP = 0.35               # ignore the upper part of the image (walls, furniture)
DEFAULT_DARK_V = 75
MAX_DARK_S = 110
MIN_AREA_FRAC = 0.0015       # of the work image
MAX_AREA_FRAC = 0.45
ASPECT_MIN, ASPECT_MAX = 0.7, 4.0   # w / h
MIN_FILL = 0.35              # dark pixels / box area (solid body, not a frame)
SCORE_MIN_LEARNED = 0.55
HIST_BINS = 32


class TargetDetector:
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = None            # dict once learned
        self.seen = False
        self.bbox = None             # (x, y, w, h) in full-frame pixels
        self.score = 0.0
        self.est_cm = None           # distance from box width (needs a learned model)
        self.offset = 0.0            # box centre: -1 left .. 0 centre .. +1 right
        self._hits = collections.deque(maxlen=5)
        self._last = None            # last raw candidate, for learn()
        self._err_logged = False
        self._load()

    # ---------- persistence ----------
    def _load(self):
        try:
            with open(self.model_path) as f:
                m = json.load(f)
            m["hist"] = np.array(m["hist"], dtype=np.float32)
            self.model = m
            logger.info("target model loaded from %s", self.model_path)
        except FileNotFoundError:
            pass
        except Exception as e:
            logger.warning("target model unreadable (%s) — re-learn it", e)

    def _save(self):
        m = dict(self.model)
        m["hist"] = m["hist"].tolist()
        tmp = self.model_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(m, f, indent=1)
        os.replace(tmp, self.model_path)

    def forget(self):
        self.model = None
        try:
            os.remove(self.model_path)
        except FileNotFoundError:
            pass

    # ---------- core ----------
    @staticmethod
    def _hist(gray_patch):
        h = cv2.calcHist([gray_patch], [0], None, [HIST_BINS], [0, 256])
        return cv2.normalize(h, h).flatten()

    def _candidates(self, frame):
        H, W = frame.shape[:2]
        scale = WORK_W / W
        small = cv2.resize(frame, (WORK_W, int(H * scale)), interpolation=cv2.INTER_AREA)
        sh, sw = small.shape[:2]
        y0 = int(sh * ROI_TOP)
        roi = small[y0:]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        dark_v = self.model["dark_v"] if self.model else DEFAULT_DARK_V
        mask = ((hsv[..., 2] < dark_v) & (hsv[..., 1] < MAX_DARK_S)).astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)            # drop speckles
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)  # join body parts

        area_img = sw * sh
        out = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            a = w * h
            # cut off by exactly one side of the frame = furniture/wall at the
            # edge, not a car in the path (a car right up close spans both)
            touches_l, touches_r = x <= 1, x + w >= sw - 1
            if touches_l != touches_r:
                continue
            if not (MIN_AREA_FRAC * area_img <= a <= MAX_AREA_FRAC * area_img):
                continue
            aspect = w / float(h)
            if not (ASPECT_MIN <= aspect <= ASPECT_MAX):
                continue
            fill = cv2.countNonZero(mask[y:y + h, x:x + w]) / float(a)
            if fill < MIN_FILL:
                continue
            hist = self._hist(gray[y:y + h, x:x + w])
            if self.model:
                score = float(cv2.compareHist(self.model["hist"], hist, cv2.HISTCMP_CORREL))
                # size sanity: much bigger/smaller than plausible -> penalise
                if self.model.get("aspect"):
                    score -= 0.3 * min(1.0, abs(aspect - self.model["aspect"]) / self.model["aspect"])
            else:
                score = 0.5 * fill + 0.5 * (1.0 - float(np.mean(gray[y:y + h, x:x + w])) / 255)
            cx = (x + w / 2) / sw * 2 - 1
            # prefer things in the driving path
            score -= 0.25 * max(0.0, abs(cx) - 0.4)
            out.append({
                "bbox": (int(x / scale), int((y + y0) / scale), int(w / scale), int(h / scale)),
                "w_small": w, "aspect": aspect, "hist": hist, "score": score,
                "offset": cx,
                "median_v": float(np.median(hsv[y:y + h, x:x + w][..., 2][mask[y:y + h, x:x + w] > 0]))
                            if cv2.countNonZero(mask[y:y + h, x:x + w]) else 255.0,
            })
        out.sort(key=lambda d: d["score"], reverse=True)
        return out

    def process(self, frame):
        """Run detection on a raw (unannotated) frame. Call once per frame."""
        try:
            cands = self._candidates(frame)
            best = cands[0] if cands else None
            self._last = best
            thr = SCORE_MIN_LEARNED if self.model else 0.55
            hit = best is not None and best["score"] >= thr
            self._hits.append(hit)
            self.seen = sum(self._hits) >= 3
            if hit:
                self.bbox, self.score, self.offset = best["bbox"], best["score"], best["offset"]
                k = self.model.get("k_cm_px") if self.model else None
                self.est_cm = (k / best["w_small"]) if k else None
            elif not self.seen:
                self.bbox, self.score, self.est_cm = None, 0.0, None
        except Exception as e:
            if not self._err_logged:
                logger.warning("target detection failed: %s", e)
                self._err_logged = True
            self.seen, self.bbox = False, None

    def draw(self, frame):
        if self.seen and self.bbox:
            x, y, w, h = self.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (80, 220, 120), 2)
            label = f"TARGET {self.score:.2f}"
            if self.est_cm:
                label += f" ~{self.est_cm:.0f}cm"
            cv2.putText(frame, label, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (80, 220, 120), 1, cv2.LINE_AA)
        return frame

    def learn(self, frame, dist_cm):
        """Learn the car currently straight ahead. Returns (ok, message)."""
        if frame is None:
            return False, "no camera frame"
        saved, self.model = self.model, None     # search without bias from an old model
        try:
            cands = [c for c in self._candidates(frame) if abs(c["offset"]) < 0.5]
        finally:
            self.model = saved
        if not cands:
            return False, "no dark car-like object near the image centre"
        # the car straight ahead: closest to the centre line, then the larger one
        c = min(cands, key=lambda d: abs(d["offset"]) - 0.002 * d["w_small"])
        model = {
            "hist": c["hist"],
            "aspect": round(c["aspect"], 3),
            "dark_v": int(min(110, max(40, c["median_v"] + 30))),
            "k_cm_px": None,
            "learn_w_px": c["w_small"],
        }
        if dist_cm and 20 <= dist_cm <= 300:
            model["k_cm_px"] = float(dist_cm * c["w_small"])
            model["learn_dist_cm"] = int(dist_cm)
        self.model = model
        self._hits.clear()
        try:
            self._save()
        except OSError as e:
            return True, f"learned (not saved: {e})"
        msg = f"learned: box {c['bbox'][2]}x{c['bbox'][3]} px"
        msg += f", distance calibrated at {int(dist_cm)} cm" if model["k_cm_px"] else \
               ", no ultrasonic distance (camera range estimate off)"
        return True, msg

    def status(self):
        return {
            "learned": self.model is not None,
            "seen": self.seen,
            "score": round(self.score, 2),
            "est_cm": None if self.est_cm is None else int(self.est_cm),
            "offset": round(self.offset, 2),
        }
