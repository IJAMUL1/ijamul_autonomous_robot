import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ijamul_description = get_package_share_directory("ijamul_description")
    ijamul_description_prefix = get_package_prefix("ijamul_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_arg = DeclareLaunchArgument(
                                        name="model", 
                                        default_value=os.path.join(
                                                                    ijamul_description, "urdf", "ijamul.urdf.xacro"
                                                                ),
                                        description="Absolute path to robot urdf file"
                                    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("ijamul_description"), "worlds", "arena.world"]
    )

    model_path = os.path.join(ijamul_description, "models")
    model_path += pathsep + os.path.join(ijamul_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = ParameterValue(
                                        Command(["xacro ", LaunchConfiguration("model")]),
                                        value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                                        os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
                                    ),
        launch_arguments={'world': world_path}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", 
                       executable="spawn_entity.py",
                       arguments=["-entity", "ijamul",
                                   "-topic", "robot_description",
                                    "-x", "-4.0",  # X position in meters
                                    "-y", "-3.0",  # Y position in meters
                                    "-z", "0.0",  # Z position in meters (usually 0 if on the ground)
                                    "-Y", "0.0",  # Yaw in radians (rotation around Z-axis)

                                  ],
                       output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ijamul_description, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        # rviz_node
    ])

