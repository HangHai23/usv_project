from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration("port")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("usv_imu_driver"), "rviz", "imu_marker.rviz"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/myimu"),
            Node(
                package="usv_imu_driver",
                executable="imu_node",
                name="imu_driver",
                output="screen",
                parameters=[{"port": port}],
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
