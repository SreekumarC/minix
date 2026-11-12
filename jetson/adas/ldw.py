"""Lane departure warning using classical computer vision.

No neural network. Edges -> Hough lines -> split by slope -> compare
the lane midpoint against the image centre. Deliberately simple: it is
explainable on a slide, which matters more here than robustness.
"""

import cv2
import numpy as np


class LDW:
    def __init__(self, offset_warn=0.18):
        self.offset_warn = offset_warn
        self.offset = 0.0        # -1 (far left) .. +1 (far right)
        self.departing = False
        self.lanes_seen = False

    def process(self, frame):
    
        """Annotate frame in place; return the same frame."""
        h, w = frame.shape[:2]

        # region of interest: lower 45% of the frame
        y0 = int(h * 0.55)
        roi = frame[y0:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 60, 160)

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=40,
            minLineLength=int(w * 0.10), maxLineGap=40,
        )

        left_x, right_x = [], []
        if lines is not None:
            # HoughLinesP's return shape is (N, 1, 4) on most OpenCV builds
            # but (N, 4) on some — reshape(-1, 4) works either way instead
            # of assuming lines[:, 0], which silently unpacked wrong on
            # this build and crashed the unpacking below.
            for x1, y1, x2, y2 in lines.reshape(-1, 4):
                if x2 == x1:
                    continue
                slope = (y2 - y1) / float(x2 - x1)
                if abs(slope) < 0.35:          # ignore near-horizontal
                    continue
                xm = (x1 + x2) / 2.0
                if slope < 0 and xm < w * 0.60:
                    left_x.append(xm)
                    cv2.line(roi, (x1, y1), (x2, y2), (255, 190, 60), 2)
                elif slope > 0 and xm > w * 0.40:
                    right_x.append(xm)
                    cv2.line(roi, (x1, y1), (x2, y2), (255, 190, 60), 2)

        self.lanes_seen = bool(left_x and right_x)

        if self.lanes_seen:
            centre = (np.mean(left_x) + np.mean(right_x)) / 2.0
            self.offset = (centre - w / 2.0) / (w / 2.0)
            self.departing = abs(self.offset) > self.offset_warn

            cx = int(centre)
            colour = (60, 60, 235) if self.departing else (140, 220, 90)
            cv2.line(frame, (cx, y0), (cx, h), colour, 2)
        else:
            self.offset = 0.0
            self.departing = False

        # fixed vehicle-centre reference
        cv2.line(frame, (w // 2, h - 24), (w // 2, h), (220, 220, 220), 1)
        return frame

    def status(self):
        if not self.lanes_seen:
            return "no lane"
        return "departure" if self.departing else "in lane"
