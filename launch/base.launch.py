from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='interplanetar_rover', executable='gui_node',
             name='gui_node', output='screen', emulate_tty=True),
    ])