from os import path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = path.join(
        get_package_share_directory('yocs_velocity_smoother'),
        'config',
        'params.yaml'
    )
    node_name = LaunchConfiguration("node_name", default="velocity_smoother")
    config_file = LaunchConfiguration("config_file", default=config)
    raw_cmd_vel_topic = LaunchConfiguration("raw_cmd_vel_topic", default="cmd_vel")
    smooth_cmd_vel_topic = LaunchConfiguration("smooth_cmd_vel_topic", default="cmd_vel/smooth")
    robot_cmd_vel_topic = LaunchConfiguration("robot_cmd_vel_topic", default="cmd_vel")
    odom_topic = LaunchConfiguration("odom_topic", default="odom")

    return LaunchDescription([
        Node(
            package='yocs_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[config_file],
            output='screen',
            remappings=[
                ([node_name, '/raw_cmd_vel'], raw_cmd_vel_topic),
                ([node_name, '/smooth_cmd_vel'], smooth_cmd_vel_topic),
                ([node_name, '/robot_cmd_vel'], robot_cmd_vel_topic),
                ([node_name, '/odometry'], odom_topic)
            ]
        )
    ])
