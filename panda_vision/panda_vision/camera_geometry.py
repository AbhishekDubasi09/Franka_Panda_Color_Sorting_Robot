"""Pixel -> world-plane back-projection from calibrated geometry.

Replaces the hand-tuned depth (`Z = 0.1`), axis scale (`* -10`) and per-colour
offsets in the original detector: a pixel defines a ray through the camera
centre, and the object position is where that ray meets the table plane.
"""
import numpy as np


def intrinsics_from_fov(width, height, horizontal_fov):
    """Pinhole intrinsics for square pixels (what Gazebo cameras produce)."""
    fx = (width / 2.0) / np.tan(horizontal_fov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def pixel_rays(pixels, fx, fy, cx, cy, R_cam_from_optical):
    """Unit-less ray directions in the camera link frame.

    `R_cam_from_optical` rotates optical-frame vectors (x right, y down,
    z forward) into the camera link frame, so any mounting convention can be
    handled by one matrix.
    """
    px = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    d_opt = np.stack([(px[:, 0] - cx) / fx,
                      (px[:, 1] - cy) / fy,
                      np.ones(len(px))], axis=1)
    return d_opt @ np.asarray(R_cam_from_optical).T


def intersect_plane(T_base_from_cam, rays_cam, plane_z):
    """Intersect camera rays with the horizontal plane z = plane_z (base frame).

    T_base_from_cam is the 4x4 pose of the camera link in the base frame.
    Returns an (N, 3) array of base-frame points.
    """
    T = np.asarray(T_base_from_cam, dtype=np.float64)
    origin = T[:3, 3]
    dirs = rays_cam @ T[:3, :3].T
    if np.any(np.abs(dirs[:, 2]) < 1e-9):
        raise ValueError("ray parallel to table plane")
    t = (plane_z - origin[2]) / dirs[:, 2]
    if np.any(t <= 0):
        raise ValueError("table plane is behind the camera")
    return origin + dirs * t[:, None]


# Gazebo camera mounted as in panda_description/urdf/sensors.xacro:
# sensor pose (0 0 0 0 1.57 0) inside camera_link, so the image looks along
# -Z of camera_link with image-right = -Y and image-up = +X.
R_CAM_FROM_OPTICAL_TOP_DOWN = np.array([
    [0.0, -1.0, 0.0],   # optical x (image right) -> camera -Y
    [-1.0, 0.0, 0.0],   # optical y (image down)  -> camera -X
    [0.0, 0.0, -1.0],   # optical z (forward)     -> camera -Z
]).T
