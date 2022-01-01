import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('particle_filter'),
            'config',
            'landmark_ground_truth.yaml'
      )

    pf_landmark_detector =  Node(
            package='particle_filter',
            executable='landmark_detector.py',
            name='landmark_detector'
        )
    visualizer =  Node(
            package='particle_filter',
            executable='visualizer.py',
            name='visualizer'
        )
    tb3_teleop = Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name = 'tb3_teleop_keyboard',
            output='screen'
            # remappings=[
            #     ('/cmd_vel', '/wheel_commands'),
        )
    rviz_config = os.path.join(
            get_package_share_directory('particle_filter'),
            'rviz',
            'particle_filter.rviz'
        )
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)]
        )
    
    return LaunchDescription([
        rviz,
        pf_landmark_detector,
        visualizer
    ])