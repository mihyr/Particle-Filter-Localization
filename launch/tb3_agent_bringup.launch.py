import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    tb3_bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('turtlebot3_bringup'), 'launch'),
         '/robot.launch.py'])
      )
    tb3_teleop = Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name = 'tb3_teleop_keyboard',
            output='screen'
        )
    pf_landmark_detector =  Node(
            package='particle_filter',
            executable='landmark_detector.py',
            name='landmark_detector'
        )

    return LaunchDescription([
        tb3_bringup,
        tb3_teleop,
        pf_landmark_detector
    ])