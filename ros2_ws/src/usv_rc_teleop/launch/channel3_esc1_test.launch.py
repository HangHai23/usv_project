from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_path(package_share: Path, package_name: str, filename: str) -> str:
    """Prefer editable workspace YAML and fall back to the installed copy."""
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / package_name / "config" / filename
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / filename)


def generate_launch_description():
    teleop_share = Path(get_package_share_directory("usv_rc_teleop"))
    receiver_share = Path(get_package_share_directory("usv_crsf_receiver"))
    actuator_share = Path(get_package_share_directory("usv_pwm_actuator"))
    receiver_config = config_path(
        receiver_share, "usv_crsf_receiver", "crsf_receiver.yaml"
    )
    differential_config = config_path(
        teleop_share, "usv_rc_teleop", "rc_differential.yaml"
    )
    control_mode_config = config_path(
        teleop_share, "usv_rc_teleop", "control_mode.yaml"
    )
    actuator_config = config_path(
        actuator_share, "usv_pwm_actuator", "pwm_actuator.yaml"
    )
    return LaunchDescription(
        [
            LogInfo(msg=f"Live PWM config: {actuator_config}"),
            DeclareLaunchArgument("receiver_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("pwm_backend", default_value="i2c"),
            DeclareLaunchArgument("pwm_port", default_value="/dev/ttyTHS2"),
            DeclareLaunchArgument("pwm_i2c_bus", default_value="7"),
            DeclareLaunchArgument("pwm_i2c_address", default_value="45"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("start_rc_nodes", default_value="true"),
            DeclareLaunchArgument(
                "force_automatic_without_rc", default_value="false"
            ),
            DeclareLaunchArgument("emergency_stop_enabled", default_value="true"),
            Node(
                package="usv_crsf_receiver",
                executable="crsf_receiver_node",
                name="crsf_receiver",
                output="screen",
                parameters=[
                    receiver_config,
                    {"port": LaunchConfiguration("receiver_port")},
                ],
                condition=IfCondition(LaunchConfiguration("start_rc_nodes")),
            ),
            Node(
                package="usv_rc_teleop",
                executable="differential_mixer",
                name="rc_differential_mixer",
                output="screen",
                parameters=[
                    differential_config
                ],
                condition=IfCondition(LaunchConfiguration("start_rc_nodes")),
            ),
            Node(
                package="usv_rc_teleop",
                executable="control_mode_arbiter",
                name="control_mode_arbiter",
                output="screen",
                parameters=[
                    control_mode_config,
                    {
                        "force_automatic_without_rc": LaunchConfiguration(
                            "force_automatic_without_rc"
                        )
                    },
                ],
            ),
            Node(
                package="usv_pwm_actuator",
                executable="pwm_actuator_node",
                name="pwm_actuator",
                output="screen",
                parameters=[
                    actuator_config,
                    {
                        "backend": LaunchConfiguration("pwm_backend"),
                        "port": LaunchConfiguration("pwm_port"),
                        "i2c_bus": LaunchConfiguration("pwm_i2c_bus"),
                        "i2c_address": LaunchConfiguration("pwm_i2c_address"),
                        "dry_run": LaunchConfiguration("dry_run"),
                        "emergency_stop_enabled": LaunchConfiguration(
                            "emergency_stop_enabled"
                        ),
                    },
                ],
            ),
        ]
    )
