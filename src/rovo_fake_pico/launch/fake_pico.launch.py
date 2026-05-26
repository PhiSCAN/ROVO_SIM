"""Launch rovo_fake_pico standalone."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("listen_port", default_value="8887"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel_gcs"),
        Node(
            package="rovo_fake_pico",
            executable="fake_pico_node",
            name="rovo_fake_pico",
            parameters=[{
                "listen_port": LaunchConfiguration("listen_port"),
                "output_topic": LaunchConfiguration("output_topic"),
                "use_sim_time": True,
            }],
            output="screen",
        ),
    ])
