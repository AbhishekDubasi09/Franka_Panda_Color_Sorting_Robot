"""Self-calibrated pixel -> table-plane mapping.

Objects sit on a plane (the table), so the image-to-robot-base mapping is a
homography. It is fitted from correspondences (pixel, base-frame xy) instead
of hand-tuned depth, scale and per-colour offsets, and needs neither camera
intrinsics nor extrinsics.
"""
import json

import cv2
import numpy as np


class TableHomography:
    def __init__(self, H, table_z=0.0, reproj_rmse=None, n_inliers=None):
        self.H = np.asarray(H, dtype=np.float64)
        self.table_z = float(table_z)
        self.reproj_rmse = reproj_rmse
        self.n_inliers = n_inliers

    @classmethod
    def fit(cls, pixels, base_xy, table_z=0.0, ransac_thresh_m=0.005):
        """Fit from >= 4 (pixel, base xy in metres) pairs, RANSAC-robust."""
        pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
        base_xy = np.asarray(base_xy, dtype=np.float64).reshape(-1, 2)
        if len(pixels) < 4 or len(pixels) != len(base_xy):
            raise ValueError("need >= 4 matching pixel/base correspondences")
        H, mask = cv2.findHomography(
            pixels, base_xy, cv2.RANSAC, ransac_thresh_m)
        if H is None:
            raise RuntimeError("homography fit failed (degenerate points?)")
        inl = mask.ravel().astype(bool)
        # Refit on inliers only with least squares for the final estimate.
        H, _ = cv2.findHomography(pixels[inl], base_xy[inl], 0)
        model = cls(H, table_z, n_inliers=int(inl.sum()))
        err = np.linalg.norm(model.pixel_to_base(pixels[inl]) - base_xy[inl],
                             axis=1)
        model.reproj_rmse = float(np.sqrt(np.mean(err ** 2)))
        return model

    def pixel_to_base(self, pixels):
        """(N,2) or (2,) pixels -> same-shape base-frame xy in metres."""
        pts = np.asarray(pixels, dtype=np.float64)
        single = pts.ndim == 1
        pts = pts.reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(pts, self.H).reshape(-1, 2)
        return out[0] if single else out

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"H": self.H.tolist(), "table_z": self.table_z,
                       "reproj_rmse": self.reproj_rmse,
                       "n_inliers": self.n_inliers}, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d["H"], d.get("table_z", 0.0), d.get("reproj_rmse"),
                   d.get("n_inliers"))
