"""PDF-documented Yahboom IMU RViz launch file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_path(package_share: Path, package_name: str, filename: str) -> str:
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / package_name / "config" / filename
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / filename)


def generate_launch_description():
    package_share = Path(get_package_share_directory("usv_imu_driver"))
    rviz_config = str(package_share / "rviz" / "imu.rviz")
    filter_config = config_path(
        package_share, "usv_imu_driver", "imu_filter_param.yaml"
    )

    return LaunchDescription(
        [
            LogInfo(msg=f"Live IMU filter config: {filter_config}"),
            DeclareLaunchArgument("port", default_value="/dev/myimu"),
            DeclareLaunchArgument("use_filter", default_value="true"),
            Node(
                package="usv_imu_driver",
                executable="imu_node",
                name="imu_driver",
                output="screen",
                parameters=[{"port": LaunchConfiguration("port")}],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick",
                output="screen",
                parameters=[filter_config],
                condition=IfCondition(LaunchConfiguration("use_filter")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
