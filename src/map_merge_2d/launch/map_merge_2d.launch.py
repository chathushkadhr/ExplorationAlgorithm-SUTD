import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("map_merge_2d")

    # Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='robot1',
        description='Host Name / Namespace')

    # Create Launch configurations
    namespace = LaunchConfiguration('namespace')
    
    # Topic remappings
    remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]

    # Declare launch actions
    start_map_merger = Node(
        package='map_merge_2d',
        executable='map_merger',
        name='map_merge',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[
          ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
        # prefix=['xterm -e gdb -ex run --args'],
        # arguments=['--ros-args', '--log-level', 'debug'],
        emulate_tty=True)
    
    start_hostmap_static_tf_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name = "hostmap_tf_publisher",
        namespace=namespace,
        remappings=remappings,
        arguments = ["0", "0", "0", "0", "0", "0", [namespace,"/map"], "map"])
 
    # Create Launch description
    ld = LaunchDescription()
    
    ld.add_action(declare_arg_namespace)

    ld.add_action(start_map_merger)
    ld.add_action(start_hostmap_static_tf_publisher)
    return ld
