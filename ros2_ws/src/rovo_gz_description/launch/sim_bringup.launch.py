"""Full simulation bringup: Zenoh router + clearpath/empty world + RViz +
rovo_fake_pico.

Dispatches to rovo_clearpath.launch.py which handles both the package-local
empty world and clearpath_gz worlds (warehouse, pipeline, office, orchard,
solar_farm, construction).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rovo_gz_share = get_package_share_directory("rovo_gz_description")
    fake_pico_share = get_package_share_directory("rovo_fake_pico")

    zenoh_router = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="rmw_zenohd",
        output="screen",
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rovo_gz_share, "launch", "rovo_clearpath.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "model": LaunchConfiguration("model"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "actors": LaunchConfiguration("actors"),
        }.items(),
    )

    fake_pico = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fake_pico_share, "launch", "fake_pico.launch.py")
        ),
        launch_arguments={
            "listen_port": LaunchConfiguration("fake_pico_port"),
            "output_topic": "/cmd_vel_gcs",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="empty",
            description="World name. 'empty' uses rovo_gz_description's "
                        "empty.sdf. Clearpath: warehouse | pipeline | "
                        "office | orchard | solar_farm | construction."
        ),
        DeclareLaunchArgument("model", default_value="tracked"),
        DeclareLaunchArgument("fake_pico_port", default_value="8887"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "actors", default_value="true",
            description="Spawn animated walking actors (warehouse only)"),
        zenoh_router,
        world_launch,
        fake_pico,
    ])
