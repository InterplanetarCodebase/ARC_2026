from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_camera_receiver', default_value='false'),
        DeclareLaunchArgument('camera_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('camera_ports_csv',
                              default_value='5000,5001,5002,5003,5004,5005,5006,5007'),
        DeclareLaunchArgument('camera_width', default_value='1920'),
        DeclareLaunchArgument('camera_height', default_value='1080'),
        DeclareLaunchArgument('camera_control_port', default_value='7000'),
        DeclareLaunchArgument('camera_rtsp_csv', default_value=''),
        DeclareLaunchArgument('camera_rtsp_latency_ports_csv', default_value=''),
        Node(package='interplanetar_rover', executable='gui_node',
             name='gui_node', output='screen', emulate_tty=True),
        Node(
            package='interplanetar_rover',
            executable='camera_receiver_node',
            name='camera_receiver_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('enable_camera_receiver')),
            parameters=[{
                'host': LaunchConfiguration('camera_host'),
                'ports_csv': LaunchConfiguration('camera_ports_csv'),
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'control_port': LaunchConfiguration('camera_control_port'),
                'rtsp_csv': LaunchConfiguration('camera_rtsp_csv'),
                'rtsp_latency_ports_csv': LaunchConfiguration('camera_rtsp_latency_ports_csv'),
            }],
        ),
    ])