from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    script = PathJoinSubstitution([
        FindPackageShare('create_bringup'),
        'scripts',
        'undockinginf.py'
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script],
            output='screen'
        )
    ])
