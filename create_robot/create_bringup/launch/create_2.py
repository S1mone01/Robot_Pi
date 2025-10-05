from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ------------------------------
    # Dichiarazione argomenti
    # ------------------------------
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
        default_value='true'
    )

    foxglove_arg = DeclareLaunchArgument(
        name='foxglove',
        default_value='false',
        description='Abilita (true) o disabilita (false) foxglove_bridge'
    )

    camera_arg = DeclareLaunchArgument(
        name='camera',
        default_value='true',
        description='Abilita (true) o disabilita (false) la telecamera'
    )

    navigation_arg = DeclareLaunchArgument(
        name='navigation',
        default_value='false',
        description='Abilita (true) o disabilita (false) Nav2'
    )

    rosbridge_arg = DeclareLaunchArgument(
        name='rosbridge',
        default_value='false',
        description='Abilita (true) o disabilita (false) rosbridge_server'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value='map_save.yaml',
        description='Nome del file della mappa (preso dalla cartella map del pacchetto create_bringup)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usa /clock simulato se true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('create_bringup'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Percorso al file dei parametri Nav2'
    )

    # ------------------------------
    # Configurazioni
    # ------------------------------
    map_file = LaunchConfiguration('map')
    map_yaml_file = PathJoinSubstitution([
        FindPackageShare('create_bringup'),
        'map',
        map_file
    ])
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # ------------------------------
    # Nodi robot
    # ------------------------------
    create_driver_node = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'robot_model': 'CREATE_2'}
        ]
    )

    include_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('create_description'),
            'launch',
            'create_2.launch'
        ])),
        condition=IfCondition(LaunchConfiguration('desc'))
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
        output='screen',
    )

    combined_button_listener_node = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('create_bringup'),
                'scripts',
                'combined_button_listener.py'
            ])
        ],
        output='screen'
    )

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
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_compressed',
        arguments=['raw', 'compressed'],
        remappings=[('in', '/camera_node/image_raw'),
                    ('out/compressed', '/camera_node/image_raw/compressed')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera'))
    )

    foxglove_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('foxglove')),
        launch_arguments={
            'port': TextSubstitution(text='8765'),
            'host': TextSubstitution(text='0.0.0.0')
        }.items()
    )

    # ------------------------------
    # Rosbridge (solo se rosbridge:=true)
    # ------------------------------
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ),
        launch_arguments={
            'port': '9090',
            'address': '0.0.0.0'
        }.items(),
        condition=IfCondition(LaunchConfiguration('rosbridge'))
    )

    # ------------------------------
    # Nav2 (solo se navigation:=true)
    # ------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py'
        ),
        condition=IfCondition(LaunchConfiguration('navigation')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    navigation_launch = TimerAction(
        period=10.0,
        condition=IfCondition(LaunchConfiguration('navigation')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    '/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py'
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file
                }.items()
            )
        ]
    )

    # ------------------------------
    # LaunchDescription finale
    # ------------------------------
    return LaunchDescription([
        config_arg,
        desc_arg,
        foxglove_arg,
        camera_arg,
        navigation_arg,
        rosbridge_arg,
        declare_map_file,
        declare_use_sim_time,
        declare_params_file,
        create_driver_node,
        include_description,
        lidar_node,
        bumper_pointcloud_node,
        combined_button_listener_node,
        camera_node,
        republish_node,
        foxglove_bridge,
        rosbridge_launch,
        localization_launch,
        navigation_launch
    ])
