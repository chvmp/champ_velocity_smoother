import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yocs_velocity_smoother'),
        'config',
        'params.yaml'
    )

    velocity_smoother_node = launch_ros.actions.Node(
        package='yocs_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters = [config],
        output='screen',
        remappings=[('velocity_smoother/raw_cmd_vel', '/raw_cmd_vel'),
                    ('velocity_smoother/smooth_cmd_vel', '/smooth_cmd_vel'),
                    ('velocity_smoother/robot_cmd_vel', '/robot_cmd_vel'),
                    ('velocity_smoother/odometry', '/odom')]
    )

    return launch.LaunchDescription([
        velocity_smoother_node
    ])
