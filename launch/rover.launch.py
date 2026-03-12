from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arm_port',   default_value='/dev/ttyUSB1'),
        Node(package='interplanetar_rover', executable='wheel_bridge_node',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('wheel_port'),
                          'baud_rate': 115200, 'watchdog_timeout': 2.0}]),
        Node(package='interplanetar_rover', executable='arm_bridge_node',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('arm_port'),
                          'baud_rate': 921600, 'watchdog_timeout': 2.0,
                          'heartbeat_interval': 0.2}]),
    ])