import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("socks_bot")
    default_model_path = os.path.join(pkg_share, 'description/socks_bot.urdf')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("socks_bot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # Узел ros2_control_node с URDF и контроллерами
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            robot_controllers,
        ],
        output="both",
    )

    # Публикация трансформаций из URDF
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
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

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name="micro_ros_serial_port", default_value="/dev/ttyAMA4",
                                            description='esp32 serial port'),
        launch.actions.DeclareLaunchArgument(name="lidar_serial_port", default_value="/dev/ttyAMA5",
                                            description='lidar serial port'),

        # Запуск micro-ROS агента
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", LaunchConfiguration('micro_ros_serial_port'), "-v6"]
        ),

        # Запуск лидара
        Node(
            package='lds01rr',
            executable='lds01rr',
            parameters=[{"uart_port": LaunchConfiguration('lidar_serial_port')}]
        ),

        # Перезагрузчик платы (если нужен)
        Node(
            package='board_rebooter',
            executable="board_rebooter",
        ),

        control_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        robot_controller_spawner,
        slam_launch,
    ])

