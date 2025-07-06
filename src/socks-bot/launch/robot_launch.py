import launch
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("socks_robot_tf2")
    default_model_path = os.path.join(pkg_share, 'description/socks_bot.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name="serial_port", default="/dev/AMA2",
                                            description='esp32 serial port'),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", LaunchConfiguration('serial_port'), "-v6"]
        ),
        Node(
            package='board_rebooter',
            executable="board_rebooter",
        ),
        Node(
            package='socks_robot_tf2',
            executable='socksbot_tf2_broadcaster',
        ),

        joint_state_publisher_node,
        robot_state_publisher_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("slam_toolbox"), '/launch/online_async_launch.py']),
            launch_arguments = {'params_file': os.path.join(pkg_share, 'config/slam_config.yaml')}.items()
        )
    ])