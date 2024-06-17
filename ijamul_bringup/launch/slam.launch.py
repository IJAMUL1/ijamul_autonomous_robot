import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ijamul_navigation = get_package_share_directory("ijamul_navigation")
    robot_bring_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ijamul_bringup"),
                "launch",
                "real_robot.launch.py"
            )
        ),
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
            ('/odom', '/ijamul_controller/odom')
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ijamul_navigation, "rviz", "ijamul_slam.rviz")],
    )

    return LaunchDescription([        
        robot_bring_up,
        slam_toolbox_node,
        rviz_node        
    ])

