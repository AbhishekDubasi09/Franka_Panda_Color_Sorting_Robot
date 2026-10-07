# Changelog

## 1.1.0

- New simulated work cell (`panda_description/worlds/lab.world`), built from SDF primitives: a graphite table, three cylindrical
  pucks and one bin per colour. It is the default world. The original scene is still available as `world_name:=empty`.
- The picker releases each colour above its own bin (`drop_position_r`, `_g`, `_b`) instead of using a fixed joint pose.
- Detector evaluation on the new scene and an end-to-end sorting evaluation (`evaluate_sorting.py`).
- `world_name` and `plane_z` are launch arguments of `panda_bringup`.
- The detector processes at most 5 frames per second (`max_rate_hz`).
- Continuous integration for the vision code. Documentation moved from the README into `docs/` and rewritten to match the code.

## 1.0.0

- Camera geometry replaces the hand-tuned depth, scale and offsets: each pixel becomes a ray that is intersected with the
  table plane.
- Lighting-adaptive colour segmentation with top-face centroids.
- The picker can sort several colours in one run (`target_colors:=RGB`).
- Ground-truth evaluation of the detectors (`evaluate_detectors.py`).
- The original detector is kept as `color_detector_legacy`; the original code is at the tag `baseline-original`.
