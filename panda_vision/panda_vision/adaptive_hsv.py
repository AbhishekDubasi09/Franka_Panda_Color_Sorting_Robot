"""Lighting-adaptive colour segmentation (replaces fixed HSV bounds).

Each frame is white-balanced (gray world) and the brightness channel is
equalised before hue classification, so the hue windows stay valid when the
illumination level or tint changes. The saturation floor is derived from the
frame (Otsu) rather than hard-coded.
"""
import cv2
import numpy as np

# Hue windows in OpenCV units (0-179), centred on the pure colours.
HUE_WINDOWS = {
    "R": [(0, 8), (172, 179)],
    "G": [(45, 75)],
    "B": [(95, 130)],
}


def normalize_lighting(bgr):
    """Gray-world white balance + CLAHE on V."""
    img = bgr.astype(np.float32)
    means = img.reshape(-1, 3).mean(axis=0)
    img *= means.mean() / np.maximum(means, 1e-6)
    img = np.clip(img, 0, 255).astype(np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    hsv[..., 2] = clahe.apply(hsv[..., 2])
    return hsv


def segment_colors(bgr, min_area=20, hue_windows=HUE_WINDOWS):
    """Return {colour_id: [(cx, cy, area), ...]} in pixel coordinates."""
    hsv = normalize_lighting(bgr)
    sat = hsv[..., 1]
    s_floor, _ = cv2.threshold(sat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    s_floor = max(int(s_floor), 60)
    colourful = (sat >= s_floor) & (hsv[..., 2] >= 40)

    found = {}
    kernel = np.ones((3, 3), np.uint8)
    for cid, windows in hue_windows.items():
        mask = np.zeros(sat.shape, np.uint8)
        for lo, hi in windows:
            mask |= cv2.inRange(hsv[..., 0], lo, hi)
        mask = cv2.bitwise_and(mask, colourful.astype(np.uint8) * 255)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        n, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        found[cid] = [(float(cents[i][0]), float(cents[i][1]),
                       int(stats[i, cv2.CC_STAT_AREA]))
                      for i in range(1, n)
                      if stats[i, cv2.CC_STAT_AREA] >= min_area]
    return found
