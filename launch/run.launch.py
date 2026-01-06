from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('nokov_to_px4'), 'config', 'params.yaml'
    )
    
    nokov_to_px4_node = Node(
        package='nokov_to_px4',
        executable='nokov_to_px4',
        name='nokov_to_px4',
        output='screen',
        parameters=[config_path]
    )

    nokov_vrpn_launch= IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vrpn_client_ros'),
                'launch',
                'sample.launch.py'
            )
        )       
    )
    

    return LaunchDescription([
        nokov_vrpn_launch,
        nokov_to_px4_node,
    ])
