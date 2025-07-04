import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    my_slam_pkg_path = get_package_share_directory('my_slam')
    simulation_world_path = os.path.join(my_slam_pkg_path, 'worlds', 'new_world.sdf')
    urdf_path = os.path.join(my_slam_pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), 
        value_type=str
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo simulation
    simulation = ExecuteProcess(
        cmd=['gz', 'sim', '-r', simulation_world_path],
        output='screen',
        name='gazebo_simulation'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    # Spawn robot WITHOUT NAME to avoid namespace issues
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # REMOVED: '-name', 'my_robot',  # This causes namespace issues!
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.15'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/lidar', '/scan'),
            ('/imu', '/imu/data')
        ],
        output='screen'
    )
    
    # REMOVED joint_state_publisher - not needed with diff_drive plugin
    tf_connector = Node(
    package='my_slam',
    executable='tf_connector.py',
    name='tf_connector',
    output='screen'
)
    # Cartographer node
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(my_slam_pkg_path, 'configs'),
            '-configuration_basename', 'robot_2d.lua'
        ],
        remappings=[
            ('scan', '/scan_fixed'),
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
    
    # YOLOv8 Detector
    yolo_detector = Node(
        package='my_slam',
        executable='yoloV8.py',
        name='yolov8_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model': 'yolov8n.pt',
            'confidence_threshold': 0.5,
            'device': 'cpu'  # or 'cuda:0' for GPU
        }]
    )
    
    # Image view for YOLO
    yolo_image_view = Node(
        package='image_view',
        executable='image_view',
        name='yolo_image_view',
        remappings=[
            ('image', '/yolo/image_annotated')
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # RViz
    rviz_config_path = os.path.join(my_slam_pkg_path, 'rviz', 'slam_yolo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Shutdown handler
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    
    scan_fixer = Node(
        package='my_slam',
        executable='fix_scan_frame.py',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('scan_in', '/scan'), ('scan_out', '/scan_fixed')]
    )
    
    # Add this node to echo the /scan topic
    scan_echo = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/scan'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Core nodes
        simulation,
        robot_state_publisher_node,
        spawn_entity,
        ros_gz_bridge,
        tf_connector,
        scan_fixer,
        scan_echo,  # Add the scan echo node here
        # SLAM
        # cartographer,
        # occupancy_grid,
        
        # Vision
        yolo_detector,
        yolo_image_view,
        
        # Visualization
        rviz_node,
        
        # Handlers
        shutdown_handler
    ])