import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    loc_kha_dir = get_package_share_directory('localization_kha')
    fast_lio_dir = get_package_share_directory('fast_lio')
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')

    # 1. Launch Livox Driver
    livox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. Launch FAST-LIO
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': os.path.join(loc_kha_dir, 'config', 'mid360.yaml')}.items()
    )

    # 3. Launch ICP Localizer Node
    # DON'T FORGET: change the file name
    map_path = os.path.join(loc_kha_dir, 'maps', 'ecc305.pcd')
    
    icp_node = Node(
        package='localization_kha',
        executable='icp_node',
        name='icp_localizer',
        output='screen',
        parameters=[{'pcd_filename': map_path}]
    )

    return LaunchDescription([
        livox_node,
        fast_lio_node,
        icp_node
    ])