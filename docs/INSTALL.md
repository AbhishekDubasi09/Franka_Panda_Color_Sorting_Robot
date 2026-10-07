# Installation

This project needs ROS 2 Humble, MoveIt 2 and Gazebo (the Fortress release that ships with `ros_gz` for Humble). I built and
ran it on Ubuntu 22.04 under WSL2 on Windows 11, with the Gazebo and RViz windows shown through WSLg. Native Ubuntu 22.04
should work the same way.

## Native install

Install ROS 2 Humble by following the
[official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), then:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit python3-colcon-common-extensions python3-rosdep python3-pip git
sudo rosdep init
rosdep update
pip3 install opencv-python numpy transforms3d
```

Get the sources and build:

```bash
mkdir -p ~/panda_ws/src && cd ~/panda_ws/src
git clone https://github.com/AbhishekDubasi09/Franka_Panda_Color_Sorting_Robot.git .
cd ~/panda_ws
rosdep install --from-paths src -y --ignore-src --skip-keys=opencv_python --rosdistro=humble
colcon build
source install/setup.bash
```

These are the same steps the `Dockerfile` uses. I built the workspace this way on my own machine. Check that the packages
are visible:

```bash
ros2 pkg list | grep panda
```

You should see `panda_bringup`, `panda_controller`, `panda_description`, `panda_moveit` and `panda_vision`.

If you copy the sources from a Windows folder into WSL instead of cloning, convert the line endings or the scripts will fail
with `python3\r: No such file or directory`. See [troubleshooting](TROUBLESHOOTING.md).

## Docker

The `Dockerfile` in this repository builds an image with ROS 2 Humble, MoveIt 2 and this workspace:

```bash
docker build -t franka-color-sorter .
xhost +local:docker
docker run -it --rm --network host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw franka-color-sorter bash
```

Inside the container, run `source install/setup.bash` and then follow [usage](USAGE.md). The image needs an X11 display for
Gazebo and RViz.

I have not built or run this image myself. The `Dockerfile` is the original project's, with the repository URL changed to
this one. The image on Docker Hub that the original project's README refers to was built from the original code. It does not
contain the changes in this repository.
