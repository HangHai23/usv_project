from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('quadruped_rc_control'))
    receiver = Path(get_package_share_directory('usv_crsf_receiver'))
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('config', default_value=str(share/'config/quadruped.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(receiver/'launch/crsf_receiver.launch.py')),
                                 launch_arguments={'port': LaunchConfiguration('port')}.items()),
        Node(package='quadruped_rc_control', executable='controller', name='quadruped_rc_control',
             output='screen', parameters=[LaunchConfiguration('config'),
                 {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
