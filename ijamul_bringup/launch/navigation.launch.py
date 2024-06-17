import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ijamul_navigation = get_package_share_directory("ijamul_navigation")
    
    # Include the robot bringup launch file
    robot_bring_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ijamul_bringup"),
                "launch",
                "real_robot.launch.py"
            )
        ),
    )

    # Define the navigation parameter file
    nav_param_file = os.path.join(ijamul_navigation, "config", "navigation.yaml")

    # Include the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ijamul_navigation, "rviz", "ijamul_navigation.rviz")],
    )

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[nav_param_file]
    # )

    # # Remap cmd_vel
    # cmd_vel_remap = [('/cmd_vel', '/ijamul_controller/cmd_vel_unstamped')]
    # # Define the map file path
    # map_file = os.path.join(ijamul_navigation, "maps", "27958_map.yaml")

    # # Navigation Node
    # nav2_node = Node(
    #     package='nav2_bringup',
    #     executable='bringup_launch',
    #     name='bringup_launch',
    #     output='screen',
    #     parameters=[nav_param_file, {'map': map_file}],
    #     remappings=cmd_vel_remap
    # )

    return LaunchDescription([
        robot_bring_up,
        rviz_node,
        # robot_state_publisher_node,
        # nav2_node
    ])
