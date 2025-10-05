from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ------------------------------
    # Dichiarazione argomenti configurabili
    # ------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usa /clock simulato se true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/home/simone/robot/src/create_robot/create_bringup/config/nav2_params_mapping.yaml',
        description='Percorso al file dei parametri Nav2'
    )

    camera_arg = DeclareLaunchArgument(
        name='camera',
        default_value='true',
        description='Abilita (true) o disabilita (false) la telecamera'
    )

    config_arg = DeclareLaunchArgument(
        name='config',
        default_value=PathJoinSubstitution([
            FindPackageShare('create_bringup'),
            'config',
            'default.yaml'
        ])
    )

    desc_arg = DeclareLaunchArgument(
        name='desc',
        default_value='true',
        description='Abilita (true) o disabilita (false) la descrizione del robot'
    )

    # ------------------------------
    # Parametri
    # ------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    camera_enabled = LaunchConfiguration('camera')
    config = LaunchConfiguration('config')
    desc_enabled = LaunchConfiguration('desc')

    # ------------------------------
    # Percorsi ai pacchetti
    # ------------------------------
    slam_toolbox_dir = FindPackageShare('slam_toolbox').find('slam_toolbox')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # ------------------------------
    # Launch SLAM Toolbox
    # ------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------
    # Launch Navigation (dopo 5 secondi)
    # ------------------------------
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file
                }.items()
            )
        ]
    )

    # ------------------------------
    # Nodi hardware / robot
    # ------------------------------
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera': '/base/axi/pcie@120000/rp1/i2c@88000/ov5647@36',
            'width': 320,
            'height': 240,
            'format': 'RGB888',
            'framerate': 1,
            'controls': {}
        }],
        condition=IfCondition(camera_enabled)
    )

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_compressed',
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera_node/image_raw'),
                    ('out/compressed', '/camera_node/image_raw/compressed')],
        output='screen',
        condition=IfCondition(camera_enabled)
    )

    create_driver_node = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        output='screen',
        parameters=[
            config,
            {'robot_model': 'CREATE_2'}
        ]
    )

    include_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('create_description'),
            'launch',
            'create_2.launch'
        ])),
        condition=IfCondition(desc_enabled)
    )

    lidar_node = Node(
        package='xv11_lidar_python',
        executable='xv11_lidar',
        name='xv11_lidar',
        output='screen',
        parameters=[{'port': '/dev/lidar'}]
    )

    bumper_pointcloud_node = Node(
        package='bumper_nav',
        executable='bumper_pointcloud_node',
        name='bumper_pointcloud_node',
        output='screen'
    )

    # ------------------------------
    # Return LaunchDescription
    # ------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        camera_arg,
        config_arg,
        desc_arg,
        slam_launch,
        navigation_launch,
        camera_node,
        republish_node,
        create_driver_node,
        include_description,
        lidar_node,
        bumper_pointcloud_node
    ])

