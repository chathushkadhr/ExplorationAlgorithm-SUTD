
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Specify actions 
    start_robot1_tf_relay_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tf_merger'), 'tf_merger.launch.py')),
        launch_arguments={
                            'namespace': 'robot1',
                            }.items())

    start_robot2_tf_relay_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('tf_merger'), 'tf_merger.launch.py')),
        launch_arguments={
                            'namespace': 'robot2',
                            }.items())

    # Create the launch description and populate
    launch_description = LaunchDescription()

    launch_description.add_action(start_robot1_tf_relay_cmd)
    launch_description.add_action(start_robot2_tf_relay_cmd)
    
    return launch_description

   
