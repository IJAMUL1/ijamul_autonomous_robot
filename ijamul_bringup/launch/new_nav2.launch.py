from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Define the path to the navigation launch file
    navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # Define paths to your custom parameter and map files
    nav_param_file = '/ijamul_ws/src/ijamul_navigation/config/navigation.yaml'
    map_file = '/ijamul_ws/src/ijamul_navigation/maps/27985_map.yaml'

    # Define remappings
    remappings = [
        ('/cmd_vel', '/ijamul_controller/cmd_vel_unstamped')
    ]

    # Include the navigation launch file with parameters and remappings
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file),
            launch_arguments={
                'params_file': nav_param_file,
                'map': map_file
            }.items(),
            remappings=remappings
        )
    ])

