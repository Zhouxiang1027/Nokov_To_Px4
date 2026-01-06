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
    config_file_arg = DeclareLaunchArgument(
        'config_file',  # 参数名
        default_value=PathJoinSubstitution([
            FindPackageShare('nokov_to_px4'),  # 自动查找包路径
            'config',
            'params.yaml'  # 默认配置文件
        ]),
        description='参数配置文件路径'
    )
    nokov_topic_arg = DeclareLaunchArgument(
        'nokov_topic',
        default_value='/vrpn_client_node/drone/pose',  # 默认话题
        description='NOKOV输入话题'
    )
    
    px4_topic_arg = DeclareLaunchArgument(
        'px4_topic',
        default_value='/fmu/in/vehicle_visual_odometry',  # 默认话题
        description='PX4输出话题'
    )
    config_file = LaunchConfiguration('config_file')   
    nokov_topic = LaunchConfiguration('nokov_topic')
    px4_topic = LaunchConfiguration('px4_topic')
    
    nokov_to_px4_node = Node(
        package='nokov_to_px4',
        executable='nokov_to_px4',
        name='nokov_to_px4',
        output='screen',
        parameters=[ config_file,  # 基础配置文件
            {
                # 命令行参数覆盖配置文件的设置
                'nokov_topic': nokov_topic,
                'px4_topic': px4_topic,
            }
            ]
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
        config_file_arg,
        nokov_topic_arg,
        px4_topic_arg,
        nokov_vrpn_launch,
        nokov_to_px4_node,
    ])
