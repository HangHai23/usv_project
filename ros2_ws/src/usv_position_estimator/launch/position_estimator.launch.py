from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('usv_position_estimator'))
    config = share/'config'/'position_estimator.yaml'
    for parent in share.parents:
        candidate = parent/'src'/'usv_position_estimator'/'config'/'position_estimator.yaml'
        if candidate.is_file():
            config = candidate
            break
    return LaunchDescription([
        LogInfo(msg=f'Live position estimator config: {config}'),
        Node(package='usv_position_estimator', executable='position_estimator',
             name='usv_position_estimator', output='screen', parameters=[str(config)]),
    ])
