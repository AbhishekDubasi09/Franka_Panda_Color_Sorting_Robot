# Troubleshooting

These are the problems I ran into while building and testing this, and what fixed them.

## The arm never moves, or the picker reports `Planning failed`

Look in the launch terminal for `Failed to activate controller : arm_controller`, or in the picker for
`Can't accept new action goals. Controller is not running`. The controllers are started while Gazebo is still loading, and on a
slow machine the arm controller can fail to activate. Stop everything and launch again. In a good start the log shows
`Configured and activated arm_controller` (and the same for `joint_state_broadcaster` and `gripper_controller`).

`failed to send response to /plan_kinematic_path (timeout)` in the `move_group` log means the machine was too busy to answer in
time. Close RViz and other heavy programs and try again. Gazebo rendering, MoveIt and a screen recorder together were more than
my 16-core laptop could handle comfortably under WSL2, with a load average above 20.

## The Gazebo window stays black

If the launch log keeps repeating `Requesting list of world names`, the Gazebo server never finished loading the world. Stop
everything and launch again. Make sure no old Gazebo processes are still running first (`pgrep -af "ign gazebo"`).

## `python3\r: No such file or directory`

A script was copied from Windows with CRLF line endings. Convert it:

```bash
sed -i 's/\r$//' pick_and_place.py
```

The repository stores the files with LF endings (`.gitattributes`), so a normal `git clone` inside Linux or WSL does not have
this problem.

## `Group 'gripper' is not a chain` in the `move_group` log

This error appears at every start-up. In all my runs the gripper still worked, because it is driven through its own
controller. I did not try to remove it.

## The detector publishes nothing

For the first seconds after launch the camera transform may not be available yet, and the detector logs `TF lookup failed`.
This stops once the robot state publisher is running. If it never stops, check that `plane_z` is set (the launch file sets it)
and that the camera frame is `camera_link`.

## Pucks bounce or leave the bin

The bins are shallow and the pucks land with some speed, so a puck can bounce for a while and occasionally leave the bin. In
my four test layouts one puck out of twelve ended up on the table. Friction on the fingers and pucks is set in
`panda_description/urdf/gazebo.xacro` and `panda_description/worlds/lab.world`.

## Build problems

```bash
rosdep update
rosdep install --from-paths src -y --ignore-src --skip-keys=opencv_python --rosdistro=humble
rm -rf build install log
colcon build
```

If you build only a few packages after editing a file that was already installed, add `--allow-overriding <package>` to
`colcon build`.
