# Usage

## Run the sorting demo

Open two terminals and source the workspace in each (`source ~/panda_ws/install/setup.bash`).

```bash
# Terminal 1: Gazebo, MoveIt, controllers and the colour detector
ros2 launch panda_bringup pick_and_place.launch.py

# Terminal 2: sort red, green and blue in that order, once the first terminal has settled
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_colors:=RGB
```

Give the first terminal a minute or two to load. The picker needs the controllers and MoveIt to be up, and on a slow machine
that takes a while. If something fails at start-up, see [troubleshooting](TROUBLESHOOTING.md).

To pick a single colour, use `-p target_color:=R` (or `G`, `B`).

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `world_name` | `lab` | World to load from `panda_description/worlds/`. `lab` is the work cell with the table, pucks and three bins. `empty` is the original scene. |
| `plane_z` | `0.06` | Height of the tops of the objects in the robot's base frame, in metres. Use `0.1158` for the original scene. |

For example, `ros2 launch panda_bringup pick_and_place.launch.py world_name:=empty plane_z:=0.1158` loads the original scene.
The picker's bin positions are set for the `lab` world. I have not tuned them for the original scene, so use `empty` to
reproduce the detection numbers, not for sorting.

## Parameters

`color_detector` (publishes `R,x,y,z` strings on `/color_coordinates`, in the `panda_link0` frame):

| Parameter | Default | Meaning |
|---|---|---|
| `plane_z` | `0.0` (set by the launch file) | Height of the object tops, in metres |
| `horizontal_fov` | `1.0` | Horizontal field of view of the camera, in radians |
| `max_rate_hz` | `5.0` | Upper limit on processed frames per second |
| `min_area` | `20` | Smallest blob, in pixels, that counts as an object |
| `show_window` | `true` | Show the detections in an OpenCV window |
| `base_frame`, `camera_frame` | `panda_link0`, `camera_link` | Frames used for the ray and the output |

`pick_and_place.py`:

| Parameter | Default | Meaning |
|---|---|---|
| `target_colors` | empty | Colours to sort in order, for example `RGB`. Overrides `target_color`. |
| `target_color` | `R` | Single colour, used when `target_colors` is empty |
| `hover_above_top` | `0.384` | Height of the hover pose above the object top, in metres |
| `approach_offset` | `0.31` | Distance the hand descends from the hover pose, in metres |
| `drop_position_r`, `_g`, `_b` | see the file | Release position above each colour's bin, in the base frame |
| `legacy_coords` | `false` | Use the original detector's coordinate convention |

## Evaluation

Both scripts need a running simulation. Run the geometric detector on its own topic first:

```bash
ros2 run panda_vision color_detector --ros-args -r /color_coordinates:=/geom_coords \
  -p plane_z:=0.06 -p show_window:=false
```

Detector accuracy, against Gazebo's true puck positions (the original detector runs next to it on `/legacy_coords`):

```bash
ros2 run panda_vision color_detector_legacy --ros-args -r /color_coordinates:=/legacy_coords
ros2 run panda_vision evaluate_detectors --ros-args -p layouts:=30 -p out:=detector_eval.json
```

End-to-end sorting on random layouts, checking where each puck ends up:

```bash
python3 panda_vision/panda_vision/evaluate_sorting.py --layouts 4 --out sorting_eval.json
```

The results in `results/` came from these scripts. `results/detector_eval.json` was produced in the original scene with the
code at tag `v1.0.0`.

## Customisation

- **Colour bounds.** The hue windows are in `panda_vision/panda_vision/adaptive_hsv.py` (`HUE_WINDOWS`). The picker only accepts
  `R`, `G` and `B`, so a new colour needs a window there and a bin and drop position in the picker.
- **Bins.** Move a bin in `panda_description/worlds/lab.world` and change the matching `drop_position_*` parameter. The
  position is in the robot's base frame, with the fingertips about 8 cm above the rim of the bin.
- **Speed.** The picker limits the arm to 10 % of the maximum velocity and acceleration (`max_velocity` and
  `max_acceleration` in `pick_and_place.py`).
- **Camera.** The camera is attached to the base in `panda_description/urdf/sensors.xacro`. If you move it, the detector
  follows through TF without further changes, as long as the camera still looks straight down.
