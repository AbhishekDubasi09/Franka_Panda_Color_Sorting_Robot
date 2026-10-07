# How it works

## Packages

| Package | Role |
|---|---|
| `panda_description` | Robot model (URDF and meshes), the camera, the Gazebo worlds and the Gazebo launch file |
| `panda_controller` | `ros2_control` controller configuration and launch |
| `panda_moveit` | MoveIt 2 configuration (SRDF, kinematics, joint limits) and launch |
| `panda_vision` | Colour detector, the camera geometry and segmentation code, the evaluation scripts, tests |
| `panda_bringup` | One launch file that starts Gazebo, the controllers, MoveIt and the detector |
| `pymoveit2` | Third-party Python interface to MoveIt 2 (Andrej Orsula), with the picker in `examples/pick_and_place.py` |

## Data flow

```
Gazebo camera ---> /camera/image_raw ---> color_detector ---> /color_coordinates ---> pick_and_place.py
                                               |                  "R,x,y,z" in               |
                                          TF: camera_link          panda_link0               v
                                          to panda_link0                              MoveIt 2 + controllers
```

1. The camera is fixed to the robot base, looking straight down at the table. Gazebo publishes its image on
   `/camera/image_raw`.
2. `color_detector` segments the image by colour (`adaptive_hsv.py`) and, for each blob, builds a ray from the camera through
   the blob's centre pixel (`camera_geometry.py`). The camera pose comes from TF. The object position is the point where the
   ray meets the horizontal plane at height `plane_z`.
3. The position is published as a string such as `R,0.601,0.001,0.060` in the `panda_link0` frame.
4. `pick_and_place.py` keeps the latest position of each colour. For each colour in `target_colors` it waits for a fresh
   detection, then runs the sequence below.
5. MoveIt 2 plans each motion and the controllers execute it.

## Pick-and-place sequence

For each colour: move to a home pose, move above the object, open the gripper, descend in a straight line, close the gripper,
return to the home pose, move above that colour's bin, open the gripper, close it again, and go back to the start pose.

## Frames and numbers worth knowing

- The robot base is at world position (0.6, 0, 0.35), rotated 1.57 rad about the vertical axis (`virtual_joint` in
  `panda_description/urdf/arm.xacro`). The table top is at z = 0.35, so the base frame's z = 0 is the table surface.
- The camera is 1.0 m above the base origin and 0.6 m along its x axis, looking straight down. Its intrinsics come from the
  field of view in `sensors.xacro` (1.0 rad, 640 by 320 pixels, so fx = fy = 585.76).
- The pucks are 60 mm tall, so the top of an object is at `plane_z = 0.06` in the base frame. The detector finds the centre of
  the top face, because a shaded side face would shift the centroid.
- The gripper hovers 0.384 m above the object top and descends 0.31 m, which puts the fingertips about 3 cm above the table,
  at mid-height on a puck.

## Why geometry and not tuned constants

The original detector turned a pixel into a position with a fixed assumed depth, a scale factor and per-colour offsets. Those
were tuned for one layout. Intersecting a ray with a known plane needs only the camera pose and the object height, both of
which are properties of the scene and not tuning. The results in the README compare the two.
