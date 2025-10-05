from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(
        get_package_share_directory('create_bringup'),
        'config', 'xbox360.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[{
            'axis_linear': {'x': 1},
            'scale_linear': {'x': 0.8},
            'scale_linear_turbo': {'x': 1.0},
            'axis_angular': {'yaw': 0},
            'scale_angular': {'yaw': 2.5},
            'scale_angular_turbo': {'yaw': 5.0},
            'require_enable_button': False,
            'deadzone_linear': 0.1,
            'deadzone_angular': 0.1
        }]
    )

    button_publisher_node = ExecuteProcess(
        cmd=['python3', '/home/simone/robot/src/create_robot/create_bringup/scripts/button_publisher.py'],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        button_publisher_node
    ])
