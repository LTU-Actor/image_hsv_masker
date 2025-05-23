from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

config = os.path.join(
        get_package_share_directory('image_hsv_masker'),
        'config',
        'params.yaml'
        )

def masker():
    return Node(
        package='image_hsv_masker',
        executable='masker',
        name='masker',
        output='screen',
        parameters=[
            config
            ],
    )
    
    
def generate_launch_description():
    return LaunchDescription([
        masker()
    ])