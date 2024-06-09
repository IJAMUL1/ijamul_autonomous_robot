import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ijamul_description = get_package_share_directory("ijamul_description")
    ijamul_navigation = get_package_share_directory("ijamul_navigation")
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        ijamul_description, "urdf", "ijamul.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ijamul_navigation, "rviz", "ijamul_slam.rviz")],
    )

       # SLAM Toolbox node
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(ijamul_navigation, 'config', 'slam.yaml')],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/odom', '/odometry/filtered')
        ]
    )



    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        slam_toolbox_node
    ])