"""Standalone RealSense sensor verification — no robot, camera streams only."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                             DeclareLaunchArgument, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = "rovo_gz_description"
    pkg_share = get_package_share_directory(pkg)

    world_arg = LaunchConfiguration("world").perform(context)
    world_path = os.path.join(pkg_share, "worlds", world_arg)
    if not os.path.isfile(world_path):
        raise RuntimeError(f"World file not found: {world_path}")

    urdf_path = os.path.join(pkg_share, "urdf", "realsense_standalone.urdf")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    bridge_yaml = os.path.join(pkg_share, "config", "camera_verify_bridge.yaml")
    rviz_config = os.path.join(pkg_share, "config", "camera_verify.rviz")

    resource_paths = [os.path.dirname(pkg_share),
                      os.environ.get("GZ_SIM_RESOURCE_PATH", "")]

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(resource_paths),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen",
    )

    spawn = TimerAction(
        period=6.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "realsense_standalone",
                       "-topic", "robot_description",
                       "-z", "0.0"],
            output="screen",
        )],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
        output="screen",
    )

    depth_converter = Node(
        package="rovo_gz_description",
        executable="depth_float32_to_uint16.py",
        name="depth_float32_to_uint16",
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    rviz = TimerAction(
        period=10.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )],
    )

    return [set_gz_path, gz_sim, rsp, spawn, bridge, depth_converter, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.sdf",
                              description="World .sdf filename in worlds/"),
        OpaqueFunction(function=_launch_setup),
    ])
