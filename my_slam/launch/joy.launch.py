from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        ),
        
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'axis_linear.x': 1,  # Left stick up/down
                'axis_angular.yaw': 0,  # Left stick left/right
                'scale_linear.x': -0.3,  # Max speed (slow for mapping)
                'scale_angular.yaw': 0.8,
                'enable_button': 0,  # X button on PlayStation, A on Xbox
                'enable_turbo_button': 1,  # Circle/B button
                'scale_linear_turbo.x': -0.6,
            }],
            remappings=[('cmd_vel', '/cmd_vel')]
        )
    ])