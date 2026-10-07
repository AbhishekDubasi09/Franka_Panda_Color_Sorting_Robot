import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # World to load (panda_description/worlds/<name>.world) and the height of the object tops in the
    # base frame. 'lab' with 0.06 is the default scene; the original scene is 'empty' with 0.1158.
    world_name = DeclareLaunchArgument("world_name", default_value="lab")
    plane_z = DeclareLaunchArgument("plane_z", default_value="0.06")

    # ------------------- Gazebo -------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_description"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world_name": LaunchConfiguration("world_name")}.items()
    )

    # ------------------- Controllers -------------------
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt -------------------
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Vision Node -------------------
    vision_node = Node(
        package="panda_vision",
        executable="color_detector",
        name="color_detector",
        output="screen",
        # Top-face height of the boxes in the panda_link0 frame (metres).
        parameters=[{"plane_z": ParameterValue(LaunchConfiguration("plane_z"), value_type=float)}]
    )

    # ------------------- MoveIt Color Picker Node -------------------
    color_picker_node = Node(
        package="pymoveit2",
        executable="pick_and_place.py",
        name="pick_and_place",
        output="screen",
        parameters=[
            {"target_color": "B"}  # {"target_color": "R"}, {"target_color": "G"}
        ]
    )

    return LaunchDescription([
        world_name,
        plane_z,
        gazebo,
        controller,
        moveit,
        vision_node,
        # color_picker_node,
    ])
