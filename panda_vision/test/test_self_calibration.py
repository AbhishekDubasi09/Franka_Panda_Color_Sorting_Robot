import numpy as np
import pytest

from panda_vision.adaptive_hsv import segment_colors
from panda_vision.table_homography import TableHomography

FX = FY = 585.7
CX, CY = 320.0, 160.0


def project(points_xy, cam_pos, yaw):
    """Pinhole camera looking straight down at the table plane (z=0)."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    rel = (np.asarray(points_xy) - cam_pos[:2]) @ R.T
    h = cam_pos[2]
    return np.stack([CX + FX * rel[:, 1] / h, CY + FY * rel[:, 0] / h], axis=1)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -1.2])
@pytest.mark.parametrize("cam", [(0.6, 0.0, 1.0), (0.45, 0.1, 0.9)])
def test_homography_recovers_unknown_camera_pose(cam, yaw):
    rng = np.random.default_rng(0)
    base = rng.uniform([0.35, -0.25], [0.65, 0.25], size=(40, 2))
    pix = project(base, np.array(cam), yaw) + rng.normal(0, 0.5, (40, 2))
    model = TableHomography.fit(pix, base)

    test = rng.uniform([0.35, -0.25], [0.65, 0.25], size=(200, 2))
    err = np.linalg.norm(
        model.pixel_to_base(project(test, np.array(cam), yaw)) - test, axis=1)
    assert err.max() < 0.003  # < 3 mm on held-out points


def test_homography_rejects_outliers_and_roundtrips(tmp_path):
    rng = np.random.default_rng(1)
    base = rng.uniform([0.35, -0.25], [0.65, 0.25], size=(40, 2))
    pix = project(base, np.array([0.6, 0, 1.0]), 0.0)
    pix[:5] += 80.0  # five gross detection errors
    model = TableHomography.fit(pix, base)
    assert model.n_inliers <= 35
    assert model.reproj_rmse < 0.002

    path = tmp_path / "cal.json"
    model.save(path)
    loaded = TableHomography.load(path)
    np.testing.assert_allclose(loaded.H, model.H)


def _scene():
    img = np.full((320, 640, 3), (120, 125, 130), np.uint8)  # grey table
    boxes = {"R": (100, 80), "G": (300, 150), "B": (500, 220)}
    colours = {"R": (30, 30, 200), "G": (40, 190, 40), "B": (200, 60, 40)}
    for cid, (x, y) in boxes.items():
        img[y - 10:y + 10, x - 10:x + 10] = colours[cid]
    return img, boxes


@pytest.mark.parametrize("gain", [0.35, 0.6, 1.0, 1.4])
@pytest.mark.parametrize("tint", [(1.0, 1.0, 1.0), (0.85, 1.0, 1.15)])
def test_segmentation_survives_lighting_changes(gain, tint):
    img, boxes = _scene()
    lit = np.clip(img.astype(np.float32) * gain * np.array(tint), 0, 255)
    found = segment_colors(lit.astype(np.uint8))
    for cid, (x, y) in boxes.items():
        assert len(found[cid]) == 1, f"{cid} at gain={gain} tint={tint}"
        cx, cy, _ = found[cid][0]
        assert abs(cx - x) < 1.5 and abs(cy - y) < 1.5


def test_camera_geometry_roundtrip_matches_gazebo_mounting():
    from panda_vision import camera_geometry as cg
    fx, fy, cx, cy = cg.intrinsics_from_fov(640, 320, 1.0)
    assert abs(fx - 585.76) < 0.1 and fx == fy

    T = np.eye(4)
    T[:3, 3] = [0.6, 0.0, 1.0]          # camera_link in panda_link0
    R = cg.R_CAM_FROM_OPTICAL_TOP_DOWN
    plane_z = 0.40
    truth = np.array([[0.6, 0.2, plane_z], [0.45, -0.15, plane_z],
                      [0.7, 0.0, plane_z]])
    # forward projection: base -> camera link -> optical -> pixel
    opt = (truth - T[:3, 3]) @ R            # = R^T r for each row r
    pix = np.stack([fx * opt[:, 0] / opt[:, 2] + cx,
                    fy * opt[:, 1] / opt[:, 2] + cy], axis=1)
    got = cg.intersect_plane(T, cg.pixel_rays(pix, fx, fy, cx, cy, R), plane_z)
    np.testing.assert_allclose(got, truth, atol=1e-9)
    # image centre looks straight down at the point below the camera
    centre = cg.intersect_plane(
        T, cg.pixel_rays([[cx, cy]], fx, fy, cx, cy, R), plane_z)
    np.testing.assert_allclose(centre[0], [0.6, 0.0, plane_z], atol=1e-9)
