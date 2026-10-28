"""Launch rovo_fake_pico standalone."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("listen_port", default_value="8888"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel_gcs"),
        DeclareLaunchArgument("rpm_multiplier", default_value="1.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="rovo_fake_pico",
            executable="fake_pico_node",
            name="rovo_fake_pico",
            parameters=[{
                "listen_port": LaunchConfiguration("listen_port"),
                "output_topic": LaunchConfiguration("output_topic"),
                "rpm_multiplier": LaunchConfiguration("rpm_multiplier"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }],
            output="screen",
        ),
    ])
