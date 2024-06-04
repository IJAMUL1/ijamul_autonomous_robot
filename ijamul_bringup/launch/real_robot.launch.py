import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ijamul_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ijamul_robot_controller"),
            "launch",
            "controller.launch.py"
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
    
    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bumperbot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    # )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        robot_localization
        # joystick,
    ])