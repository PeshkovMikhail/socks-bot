import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("socks_bot")
    default_model_path = os.path.join(pkg_share, 'description/bot.xacro')
    print(os.path.join(pkg_share, 'description/meshes'))
    nodes = [
        SetEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                os.path.join(pkg_share, 'description/meshes')
        ),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim', default_value='True',
                                            description='Flag to enable use_sim'),
        launch.actions.DeclareLaunchArgument(name="micro_ros_serial_port", default_value="/dev/ttyAMA4",
                                            description='esp32 serial port'),
        launch.actions.DeclareLaunchArgument(name="lidar_serial_port", default_value="/dev/ttyAMA5",
                                            description='lidar serial port'),
    ]

    

    use_sim = LaunchConfiguration('use_sim')
    

    if not bool(use_sim):
        hardware_plugin = "socks_bot/SocksBotHardware"
        nodes.extend([
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name='micro_ros_agent',
                arguments=["serial", "--dev", LaunchConfiguration('micro_ros_serial_port'), "-v6"]
            ),
            Node(
                package='lds01rr',
                executable='lds01rr',
                parameters=[{"uart_port": LaunchConfiguration('lidar_serial_port')}]
            ),
            Node(
                package='board_rebooter',
                executable="board_rebooter",
            ),
        ])
    else:
        hardware_plugin = "gz_ros2_control/GazeboSystem"
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

        nodes.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    'gz_args': ["empty.sdf"],  # Replace with your own world file
                    'on_exit_shutdown': 'True'
                }.items(),
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'socks_bot',            # имя модели в симуляции
                    '-topic', 'robot_description', # откуда URDF брать
                    '-z', '0.1'                    # высота над землёй
                ],
                output='screen'
            ),

            # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
            # Node(
            #     package='ros_gz_bridge',
            #     executable='parameter_bridge',
            #     arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
            #     remappings=[('/example_imu_topic',
            #                 '/remapped_imu_topic'),],
            #     output='screen'
            # )
        ])


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("socks_bot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    robot_description = Command(['xacro ', LaunchConfiguration('model'), ' hardware_plugin:=', hardware_plugin])

    # Узел ros2_control_node с URDF и контроллерами
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            robot_controllers,
        ],
        output="both",
    )

    # Публикация трансформаций из URDF
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim}
        ]
    )

    # Спавнеры контроллеров
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-manager", "/controller_manager",
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), '/launch/online_async_launch.py']),
        launch_arguments={'params_file': os.path.join(pkg_share, 'config/slam_config.yaml')}.items()
    )

    nodes.extend([
        control_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        robot_controller_spawner,
        slam_launch,
    ])

    return LaunchDescription(nodes)

