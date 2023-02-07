from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    # return LaunchDescription([
       
    

    node = Node(
            package='exploration',
            # namespace='turtlesim1',
            executable='exec_exp',
            name='exp'
        )
    
    
    ld = LaunchDescription()
    ld.add_action(node)
    return ld
    
    

