import os
import launch
import launch.actions
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # 1. LiDAR to Robot Base
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )

    # 2. IMU to Robot Base
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 3. NEW: Map to Odom (The missing link that fixes the Red RViz error)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    localization_param_dir = os.path.join(
        get_package_share_directory('lidar_localization_ros2'),
        'param',
        'localization.yaml')

    # 4. The Localization Node
    lidar_localization = LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        parameters=[localization_param_dir],
        remappings=[('/cloud', '/velodyne_points')],
        output='screen'
    )

    # 5. Transition: Configure
    configure_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # 6. Transition: Activate (Triggered after configuring)
    activate_event = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="Node Inactive -> Activating..."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Add all actions to the description
    ld.add_action(lidar_tf)
    ld.add_action(imu_tf)
    ld.add_action(map_to_odom_tf) # Added this
    ld.add_action(lidar_localization)
    ld.add_action(activate_event)
    ld.add_action(configure_event)

    return ld