import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
 
def generate_launch_description():

# Launch m-explore (generates the map)      
    explore_lite_file_dir = os.path.join(get_package_share_directory('explore_lite'), 'launch')
    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([explore_lite_file_dir, '/explore.launch.py']),
        )

    return LaunchDescription([
        
        explore_lite
        
        ])