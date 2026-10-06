# Contributing

Issues and pull requests are welcome.

If you report a bug, please include your ROS 2 distribution, your Gazebo version and the command you ran.

For a pull request, a small focused change is easiest to review. If you touch the vision code, please run the
tests first: `python3 -m pytest panda_vision/test/test_self_calibration.py`.

The most useful contributions would be testing on a real Panda with a calibrated camera, other object shapes,
and cluttered tables.
