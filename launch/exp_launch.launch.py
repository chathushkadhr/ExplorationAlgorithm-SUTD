import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("exploration")

    #Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='robot1',
        description='Host Name / Namespace')
    
    declare_arg_index = DeclareLaunchArgument('index',
        default_value="1",
        description='Index of the robot')

    # Create Launch configuratios
    namespace = LaunchConfiguration('namespace')
    
    declare_arg_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    index =  LaunchConfiguration('index', default=1)
    

    remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]
    
    node1 = Node(
            package='exploration',
            # namespace='turtlesim1',
            executable='exec_exp',
            name='MWFCN_node',
            namespace=namespace,
            remappings=remappings,
            output="screen",
            parameters=[ {'use_sim_time': use_sim_time},
            ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            prefix=['xterm -e gdb -ex run --args']
        # arguments=['--ros-args', '--log-level', 'debug'],
        #emulate_tty=True)
            
        )
   
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_sim_time)
    ld.add_action(declare_arg_namespace)
    ld.add_action(declare_arg_index)
    ld.add_action(node1)

    
    return ld