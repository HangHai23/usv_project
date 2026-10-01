"""Independent single-direction ESC actuator for startup-threshold testing."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[2]
    dry_run = LaunchConfiguration("dry_run")
    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="usv_pwm_actuator",
                executable="pwm_actuator_node",
                name="pwm_actuator",
                output="screen",
                parameters=[
                    str(
                        root
                        / "ros2_ws/src/usv_pwm_actuator/config/pwm_actuator.yaml"
                    ),
                    {
                        "forward_only": True,
                        "output_limit_percent": 100.0,
                        "channel_1_trim_percent": 0.0,
                        "channel_2_trim_percent": 0.0,
                        "channel_1_unidirectional_start_us": 1000,
                        "channel_2_unidirectional_start_us": 1000,
                        "command_timeout": 0.3,
                        "dry_run": dry_run,
                        "emergency_stop_enabled": PythonExpression(
                            ["'", dry_run, "' != 'true'"]
                        ),
                    },
                ],
                remappings=[("/propulsion/command", "/esc_start_test/command")],
            ),
        ]
    )
