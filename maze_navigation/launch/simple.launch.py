import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node # Needed for map_server

def generate_launch_description():
    pkg_dir = get_package_share_directory('maze_navigation')

    # Path to your map yaml
    map_file = os.path.join(pkg_dir, 'worlds', 'map.yaml')
    
    world_file = os.path.join(pkg_dir, 'worlds', 'simple.world.xml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'simple.rviz')

    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    do_fake_localization = LaunchConfiguration('do_fake_localization')
    publish_tf_odom2baselink = LaunchConfiguration('publish_tf_odom2baselink')
    publish_log_topics = LaunchConfiguration('publish_log_topics')
    force_publish_vehicle_namespace = LaunchConfiguration('force_publish_vehicle_namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('headless', default_value='False'))
    ld.add_action(DeclareLaunchArgument('do_fake_localization', default_value='True'))
    ld.add_action(DeclareLaunchArgument('publish_tf_odom2baselink', default_value='True'))
    ld.add_action(DeclareLaunchArgument('force_publish_vehicle_namespace', default_value='False'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('publish_log_topics', default_value='False'))

    # 1. Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # 2. Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 3. Robot Navigation Node
    robot_nav_node = Node(
        package='maze_navigation',
        executable='robot_nav',
        name='robot_nav_node',
        output='screen',
        # parameters=[{'use_sim_time': True}]
    )

    # 4. Scan Projector Node
    scan_projector_node = Node(
        package='maze_navigation',
        executable='scan_projector',
        name='scan_projector_node',
        output='screen',
    )

    # Include MVSIM's base launch file
    mvsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mvsim'), 'launch', 'launch_world.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'headless': headless,
            'do_fake_localization': do_fake_localization,
            'publish_tf_odom2baselink': publish_tf_odom2baselink,
            'force_publish_vehicle_namespace': force_publish_vehicle_namespace,
            'publish_log_topics': publish_log_topics,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file
        }.items()
    )

    # Add all actions to LaunchDescription
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(mvsim_launch)
    ld.add_action(robot_nav_node)
    ld.add_action(scan_projector_node)

    return ld