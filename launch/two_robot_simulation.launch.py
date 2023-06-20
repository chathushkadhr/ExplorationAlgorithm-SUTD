
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Specify actions 
    
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true')
    

    
    start_navigation_robot1_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tb3_sim'),"launch", 'navigation.launch.py')),
        launch_arguments={ 'namespace': 'robot1', 
                          'use_sim_time':"true",
                            }.items())
    
    start_navigation_robot2_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tb3_sim'),"launch", 'navigation.launch.py')),
        launch_arguments={ 'namespace': 'robot2', 
                          'use_sim_time':"true",
                            }.items())
    
    # start_mapmerger_robot1_cmd =  IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('map_merge_2d'),"launch", 'map_merge_2d.launch.py')),
    #     launch_arguments={ 'namespace': 'robot1', 
    #                         'use_sim_time':"true",
    #                         }.items())
    
    # start_mapmerger_robot2_cmd =  IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('map_merge_2d'),"launch", 'map_merge_2d.launch.py')),
    #     launch_arguments={ 'namespace': 'robot2', 
    #                       'use_sim_time':"true",
    #                         }.items())
    
    # start_tfmerger_cmd =  IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('tf_merger'), 'htx_tf_merger.launch.py')),
    #         launch_arguments={ 
                            # }.items())

    
    
    # start_robot1_exploration_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('exploration'),"launch", 'exp_launch.launch.py')),
    #     launch_arguments={
    #                         'namespace': 'robot1', 
    #                         'index': '1',
    #                         }.items())

    # start_robot2_exploration_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('exploration'),"launch", 'exp_launch.launch.py')),
    #     launch_arguments={
    #                         'namespace': 'robot2',
    #                         'index': '2',
    #                         }.items())

    # Create the launch description and populate
    launch_description = LaunchDescription()

    launch_description.add_action(start_navigation_robot1_cmd)
    launch_description.add_action(start_navigation_robot2_cmd)
    # launch_description.add_action(start_mapmerger_robot1_cmd)
    # launch_description.add_action(start_mapmerger_robot2_cmd)
    # launch_description.add_action(start_tfmerger_cmd)
   
    
    return launch_description

   