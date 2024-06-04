import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ijamul = get_package_share_directory("ijamul_description")
    # gazebo_ros = get_package_share_directory("gazebo_ros")
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        ijamul, "urdf", "ijamul.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ijamul, "rviz", "display.rviz")],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ijamul", "-topic", "robot_description"],
        output="screen"
    )


    # # Path to the rplidar_ros package
    # rplidar_ros_package_dir = get_package_share_directory('rplidar_ros')

    # # Include the RPLidar A1 view launch file
    # view_rplidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(rplidar_ros_package_dir, 'launch', 'view_rplidar_a1_launch.py'))
    # )

    # start_gazebo_server = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(gazebo_ros, "launch", "gzserver.launch.py")])
    # )

    # start_gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(gazebo_ros, "launch", "gzclient.launch.py")])
    # )


    


    return LaunchDescription([
        model_arg,
        # start_gazebo_server,
        # start_gazebo_client,
        # rplidar_node,
        spawn_robot,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # rviz_node
    ])