import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pointcloud_downsampling'), 'config', 'pointcloud_downsampling.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pointcloud_downsampling_node = Node(
        package='pointcloud_downsampling',
        executable='pointcloud_downsampling_node',
        output='screen',
        parameters=[
            config, 
            {'use_sim_time': use_sim_time }
        ]
    )

    return LaunchDescription([pointcloud_downsampling_node])
