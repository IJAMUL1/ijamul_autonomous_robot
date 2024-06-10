import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ijamul_description = get_package_share_directory("ijamul_description")
    # ijamul_navigation = get_package_share_directory("ijamul_navigation")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("ijamul_description"),
                    "urdf",
                    "ijamul.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("ijamul_robot_controller"),
                "config",
                "ijamul_controllers.yaml",
            ),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ijamul_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # arguments=["-d", os.path.join(ijamul_navigation, "rviz", "ijamul_slam.rviz")],
        arguments=["-d", os.path.join(ijamul_description, "rviz", "display.rviz")],
    )


    

    
    return LaunchDescription([
        
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rviz_node,









        # model_arg,

        # robot_localization,  # Uncomment if you want to use robot_localization
        # mpu6050driver,
        # rplidar_a1,
        # slam_toolbox_node
        # throttle_scan_node,
        
    ])




# robot_localization = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[os.path.join(get_package_share_directory("ijamul_localization"), "config", "ekf.yaml")],
    # )
    
    # mpu6050driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("mpu6050driver"),
    #             "launch",
    #             "mpu6050driver_launch.py"
    #         )
    #     ),
    #     launch_arguments={
    #         '/imu': '/imu/data'
    #     }.items()
    # )

    # rplidar_a1 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("rplidar_ros"),
    #             "launch",
    #             "rplidar_a1_launch.py"
    #         )
    #     ),
    # )

    # # SLAM Toolbox node
    # slam_toolbox_node = Node(
    #     package="slam_toolbox",
    #     executable="sync_slam_toolbox_node",
    #     name="slam_toolbox",
    #     output="screen",
    #     parameters=[os.path.join(ijamul_navigation, 'config', 'slam.yaml')],
    #     remappings=[
    #         ('/scan', '/scan'),
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static'),
    #         ('/odom', '/odometry/filtered')
    #     ]
    # )


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
