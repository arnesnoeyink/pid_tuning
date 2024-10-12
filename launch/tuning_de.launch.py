from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tuning_de = Node(
                     package='pid_tuning', 
                     executable='tuning_node_de',
                     output='screen')

    return LaunchDescription([
        tuning_de
        ])