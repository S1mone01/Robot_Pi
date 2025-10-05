from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    node_to_run = LaunchConfiguration('node').perform(context)
    slow_value = LaunchConfiguration('slow_value').perform(context)

    # Percorso generico agli script dentro create_bringup/scripts/
    keepout_script = PathJoinSubstitution([
        FindPackageShare('create_bringup'),
        'scripts',
        'keepout_polygoninf.py'
    ])
    speed_script = PathJoinSubstitution([
        FindPackageShare('create_bringup'),
        'scripts',
        'speed_limit_polygon_publisherinf.py'
    ])

    if node_to_run == 'keepout':
        cmd = ['python3', keepout_script]
    elif node_to_run == 'speed':
        cmd = [
            'python3', speed_script,
            '--ros-args', '-p', f'slow_value:={slow_value}'
        ]
    else:
        raise RuntimeError(f"Opzione nodo sconosciuta: {node_to_run}")

    return [ExecuteProcess(
        cmd=cmd,
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node',
            default_value='keepout',
            description='Nodo da avviare: keepout o speed'
        ),
        DeclareLaunchArgument(
            'slow_value',
            default_value='25',
            description='Velocità percentuale all’interno del poligono'
        ),
        OpaqueFunction(function=launch_setup)
    ])

