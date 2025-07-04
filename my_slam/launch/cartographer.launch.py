from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_slam')
    
    # Fix scan frame
    scan_fixer = Node(
        package='my_slam',
        executable='fix_scan_frame.py',
        name='scan_frame_fixer',
        parameters=[{'scan_in' : '/scan', 'scan_out': '/scan_fixed'}],
    )
    # Cartographer
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'configs'),
            '-configuration_basename', 'robot_2d.lua'
        ],
        remappings=[
            ('scan', '/scan_fixed'),  # Use fixed scan
            ('imu', '/imu/data'),
            ('odom', '/odometry')
        ]
    )
    
    # Occupancy grid
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05}
        ]
    )
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    # TF buffer for debugging
    view_frames = Node(
        package='tf2_tools',
        executable='view_frames',
        name='view_frames',
        output='screen'
    )
    
    return LaunchDescription([
        scan_fixer,
        cartographer,
        occupancy_grid,
        static_tf,
        view_frames
    ])