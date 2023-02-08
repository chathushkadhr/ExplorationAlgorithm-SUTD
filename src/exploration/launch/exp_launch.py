import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("exploration")

    # Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='robot6',
        description='Host Name / Namespace')

    # Create Launch configurations
    namespace = LaunchConfiguration('namespace')

    
       
    

    node = Node(
            package='exploration',
            # namespace='turtlesim1',
            executable='exec_exp',
            name='MWFCN_node',
            namespace=namespace,
            parameters=[
            ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)]
        # prefix=['xterm -e gdb -ex run --args'],
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
        )
    
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(node)
    
    return ld
    
    

