import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
    
# Launch localization (localizes the robot)
    localization = Node(
            package = 'task_4',
            executable ='localization',
            name = 'localization',
            output = 'screen'
        )
    
# Launch sanitizer (starts the sanitization process)
    sanitizer = Node(
            package = 'task_4',
            executable = 'sanitizer',
            name = 'sanitizer',
            output = 'screen'
        )
 
    return LaunchDescription([
      
      localization,


      sanitizer

  ])