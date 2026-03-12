from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arm_serial_port',   default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('watchdog_timeout',  default_value='2.0'),
        Node(
            package='interplanetar_rover', executable='wheel_bridge_node.py',
            name='wheel_bridge', output='screen',
            parameters=[{
                'serial_port':      LaunchConfiguration('wheel_serial_port'),
                'serial_baud':      115200,
                'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            }]
        ),
        Node(
            package='interplanetar_rover', executable='arm_bridge_node.py',
            name='arm_bridge', output='screen',
            parameters=[{
                'serial_port':        LaunchConfiguration('arm_serial_port'),
                'serial_baud':        921600,
                'watchdog_timeout':   LaunchConfiguration('watchdog_timeout'),
                'heartbeat_interval': 0.2,
            }]
        ),
    ])
