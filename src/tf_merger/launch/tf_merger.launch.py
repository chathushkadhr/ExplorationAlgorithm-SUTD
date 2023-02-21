from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("tf_merger")

    # Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace', default_value='robot1', description='Namespace of TF topic')

    # Create Launch configurations
    namespace = LaunchConfiguration('namespace')
    
    # Topic remappings
    remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]

    # Declare launch actions
    start_tf_merger = Node(
        package='tf_merger',
        executable='tf_merger',
        name='tf_merger',
        namespace=namespace,
        output='screen',
        respawn = True,
        # remappings=remappings,
        parameters=[
                {'namespace': namespace},
            ],
        emulate_tty=True)
 
    # Create Launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_arg_namespace)
    
    ld.add_action(start_tf_merger)
    return ld