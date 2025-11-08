from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('cmd_vel_type',  default_value='stamped'),  # 'stamped' | 'twist'
        DeclareLaunchArgument('max_speed',     default_value='0.22'),
        DeclareLaunchArgument('min_speed',     default_value='0.01'),
        DeclareLaunchArgument('stop_distance', default_value='1.0'),
        DeclareLaunchArgument('turn_angle_deg',default_value='30.0'),

        Node(
            package='mahir_enes_akpinar',
            executable='hareket_kontrol',
            name='hareket_kontrol',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'cmd_vel_type':  LaunchConfiguration('cmd_vel_type'),
                'max_speed':     LaunchConfiguration('max_speed'),
                'min_speed':     LaunchConfiguration('min_speed'),
                'stop_distance': LaunchConfiguration('stop_distance'),
                'turn_angle_deg':LaunchConfiguration('turn_angle_deg'),
            }]
        )
    ])
