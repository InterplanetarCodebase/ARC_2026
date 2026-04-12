from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('arm_port',   default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('enable_camera_transmitter', default_value='false'),
        DeclareLaunchArgument('camera_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('camera_devices_csv', default_value=''),
        DeclareLaunchArgument('camera_base_port', default_value='5000'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='25'),
        DeclareLaunchArgument('camera_bitrate', default_value='1000000'),
        DeclareLaunchArgument('camera_stagger', default_value='1.5'),
        DeclareLaunchArgument('camera_skip_probe', default_value='false'),
        DeclareLaunchArgument('camera_control_port', default_value='7000'),
        Node(package='interplanetar_rover', executable='wheel_bridge_node',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('wheel_port'),
                          'baud_rate': 115200, 'watchdog_timeout': 2.0}]),
        Node(package='interplanetar_rover', executable='arm_bridge_node',
             output='screen', emulate_tty=True,
             parameters=[{'serial_port': LaunchConfiguration('arm_port'),
                          'baud_rate': 921600, 'watchdog_timeout': 2.0,
                          'heartbeat_interval': 0.2}]),
        Node(
            package='interplanetar_rover',
            executable='camera_transmitter_node',
            name='camera_transmitter_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_camera_transmitter')),
            parameters=[{
                'host': LaunchConfiguration('camera_host'),
                'cameras_csv': LaunchConfiguration('camera_devices_csv'),
                'base_port': LaunchConfiguration('camera_base_port'),
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'fps': LaunchConfiguration('camera_fps'),
                'bitrate': LaunchConfiguration('camera_bitrate'),
                'stagger': LaunchConfiguration('camera_stagger'),
                'skip_probe': LaunchConfiguration('camera_skip_probe'),
                'control_port': LaunchConfiguration('camera_control_port'),
            }],
        ),
    ])
