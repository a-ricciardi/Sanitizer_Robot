import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_config_file = os.path.join(get_package_share_directory('task_1'), 'config', 'nav2_config.yaml')
    map_config_file = os.path.join(get_package_share_directory('task_1'), 'config', 'my_map.yaml')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    rviz_config_file = os.path.join(get_package_share_directory('task_1'), 'config', 'nav2_default_view.rviz')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')   
    
    sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

# Launch gazebo with big house
    big_house = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/turtlebot3_big_house.launch.py'])
    )

# Launch rviz2
    rviz2 = Node(
         package = 'rviz2',
         executable = 'rviz2',
         name = 'rviz2',
         arguments = ['-d', rviz_config_file],
         parameters = [{'use_sim_time': True}],
         output = 'screen')

# Launch slam
    slam = Node(
        parameters=[
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
# Launch nav2_bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_config_file,
                        'map': map_config_file}.items())

    return LaunchDescription([

        sim_time,
        
        big_house,
        
        rviz2,

        slam,

        nav2_bringup 
    
    ])