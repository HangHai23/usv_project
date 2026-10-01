from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_path(package_share: Path) -> str:
    """Prefer editable workspace YAML and fall back to the installed copy."""
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / "usv_imu_driver" / "config" / "imu_driver.yaml"
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / "imu_driver.yaml")


def generate_launch_description():
    share = Path(get_package_share_directory("usv_imu_driver"))
    config = config_path(share)
    return LaunchDescription(
        [
            LogInfo(msg=f"Live IMU config: {config}"),
            DeclareLaunchArgument("port", default_value="/dev/myimu"),
            Node(
                package="usv_imu_driver",
                executable="imu_node",
                name="imu_driver",
                output="screen",
                parameters=[config, {"port": LaunchConfiguration("port")}],
            ),
        ]
    )
