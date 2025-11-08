from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # TurtleBot3 Gazebo paketi dizini
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Gazebo dünya dosyasını dahil et
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Teleop node (klavyeden kontrol)
    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix=['xterm', '-e']  # ayrı terminalde açmak için (isteğe bağlı)
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo_world,
        teleop_node,
        rviz_node
    ])
