import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('laser2lidar'),
        'config',
        'params.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='laser2lidar',
            executable='laser2lidar_node',
            name='laser2lidar_node',
            parameters=[config_file]
        )
    ])

