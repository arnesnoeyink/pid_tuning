from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tuning_hs = Node(
                     package='pid_tuning', 
                     executable='tuning_node_hs',
                     name='tuning_node_hs',
                     output='screen')

    return LaunchDescription([
        tuning_hs
        ])