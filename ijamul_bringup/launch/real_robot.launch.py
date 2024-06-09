import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ijamul_description = get_package_share_directory("ijamul_description")
    ijamul_navigation = get_package_share_directory("ijamul_navigation")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(ijamul_description, "urdf", "ijamul.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

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

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ijamul_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ijamul_robot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("ijamul_localization"), "config", "ekf.yaml")],
    )
    
    mpu6050driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mpu6050driver"),
                "launch",
                "mpu6050driver_launch.py"
            )
        ),
        launch_arguments={
            '/imu': '/imu/data'
        }.items()
    )

    rplidar_a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_a1_launch.py"
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
            ('/odom', '/odometry/filtered')
        ]
    )
    # # Throttle node
    # throttle_scan_node = Node(
    #     package="topic_tools",
    #     executable="throttle",
    #     name="throttle_scan",
    #     arguments=["messages", "/scan", "2"],
    #     remappings=[
    #         ('/scan', '/scan_throttled')
    #     ]
    # )


    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        rviz_node,
        hardware_interface,
        controller,
        robot_localization,  # Uncomment if you want to use robot_localization
        mpu6050driver,
        rplidar_a1,
        slam_toolbox_node
        # throttle_scan_node,
        
    ])


# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     ijamul_description = get_package_share_directory("ijamul_description")
#     ijamul_navigation = get_package_share_directory("ijamul_navigation")
#     model_arg = DeclareLaunchArgument(
#         name="model", 
#         default_value=os.path.join(ijamul_description, "urdf", "ijamul.urdf.xacro"),
#         description="Absolute path to robot urdf file"
#     )

#     robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

#     controller = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("ijamul_robot_controller"),
#                 "launch",
#                 "controller.launch.py"
#             )
#         ),
#         launch_arguments={
#             "use_simple_controller": "False",
#             "use_python": "False"
#         }.items(),
#     )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description}]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", os.path.join(ijamul_navigation, "rviz", "ijamul_slam.rviz")],
    # )

    # # SLAM Toolbox node
    # slam_toolbox_node = Node(
    #     package="slam_toolbox",
    #     executable="sync_slam_toolbox_node",
    #     name="slam_toolbox",
    #     output="screen",
    #     parameters=[os.path.join(ijamul_navigation, 'config', 'slam.yaml')],
    #     remappings=[
    #         ('/scan', '/scan_throttled'),
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static'),
    #         ('/odom', '/odometry/filtered')
    #     ]
    # )

    # Throttle node
    # throttle_scan_node = Node(
    #     package="topic_tools",
    #     executable="throttle",
    #     name="throttle_scan",
    #     arguments=["messages", "/scan", "2"],
    #     remappings=[
    #         ('/scan', '/scan_throttled')
    #     ]
    # )

#     # LIDAR node (assuming rplidar_ros is used)
#     rplidar_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("rplidar_ros"),
#                 "launch",
#                 "rplidar_a1_launch.py"
#             )
#         )
#     )

#     # IMU node (assuming mpu6050driver is used)
#     imu_node = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory("mpu6050driver"),
#                 "launch",
#                 "mpu6050driver_launch.py"
#             )
#         ),
#         launch_arguments={
#             '/imu': '/imu/data'
#         }.items()
#     )

#     # # EKF Node for sensor fusion
#     # ekf_node = Node(
#     #     package='robot_localization',
#     #     executable='ekf_node',
#     #     name='ekf_filter_node',
#     #     output='screen',
#     #     parameters=[os.path.join(ijamul_navigation, 'config', 'ekf.yaml')],
#     #     remappings=[
#     #         ('/odometry/filtered', '/odom'),
#     #         ('/imu/data', '/imu/data')
#     #     ]
#     # )

#     return LaunchDescription([
#         model_arg,
#         # robot_state_publisher_node,
#         controller,
#         # slam_toolbox_node,
#         # throttle_scan_node,
#         # rviz_node,
#         rplidar_node,
#         imu_node,
#         # ekf_node
#     ])
