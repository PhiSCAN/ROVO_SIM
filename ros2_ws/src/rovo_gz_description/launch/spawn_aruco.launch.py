"""Spawn ArUco bot into an existing Gazebo Sim world.

This launch file spawns a mobile ArUco bot into a running Gazebo simulation,
creates a bridge node for cmd_vel, and launches teleop and joy nodes for controller input.

It assumes a gz server and world are already running (started by another launch file).
The world name is auto-detected from the running simulation.

Usage:
  # Spawn ArUco bot with teleop controller
  ros2 launch rovo_gz_description spawn_aruco.launch.py

  # Spawn ArUco bot at custom position
  ros2 launch rovo_gz_description spawn_aruco.launch.py x:=2.0 y:=-1.0 marker_name:=my_aruco_bot
"""
import os
import re
import subprocess
import tempfile
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _get_world_name():
    """Query running Gazebo Sim for the world name."""
    try:
        result = subprocess.run(
            ['gz', 'topic', '-l'],
            capture_output=True,
            text=True,
            timeout=5
        )
        for line in result.stdout.splitlines():
            match = re.match(r'world/([^/]+)/', line)
            if match:
                return match.group(1)
    except Exception:
        pass
    return "empty"


def _launch_setup(context, *args, **kwargs):

    # Get launch configurations
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    marker_name = LaunchConfiguration("marker_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    
    # Auto-detect world name from running simulation if not specified
    if world == "empty":
        world = _get_world_name()

    # Always spawn the ArUco bot
    model_uri = "model://mars_rover"

    # First, remove any existing model with the same name to avoid conflicts
    # Using DeleteEntity service
    remove_entity_cmd = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            f"/world/{world}/remove",
            "ros_gz_interfaces/srv/DeleteEntity",
            f"'{{\"name\": \"{marker_name}\"}}'"
        ],
        output="screen",
        shell=True,
    )

    # Spawn the model using ros_gz_sim create
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_aruco",
        arguments=[
            "-name", marker_name,
            "-x", x, "-y", y, "-z", z,
            "-R", roll, "-P", pitch, "-Y", yaw,
            "-file", model_uri,
        ],
        output="screen",
    )

    # Create bridge for cmd_vel, odom, and joint_state
    ros_cmd_vel_topic = f"/aruco/{marker_name}/cmd_vel"
    gz_cmd_vel_topic = f"world/{world}/model/{marker_name}/cmd_vel"
    ros_odom_topic = f"/aruco/{marker_name}/odom"
    gz_odom_topic = f"world/{world}/model/{marker_name}/odom"
    ros_joint_state_topic = f"/aruco/{marker_name}/joint_state"
    gz_joint_state_topic = f"world/{world}/model/{marker_name}/joint_state"
    
    bridge_yaml_content = f"""- ros_topic_name: "{ros_cmd_vel_topic}"
  gz_topic_name: "{gz_cmd_vel_topic}"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
- ros_topic_name: "{ros_odom_topic}"
  gz_topic_name: "{gz_odom_topic}"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "{ros_joint_state_topic}"
  gz_topic_name: "{gz_joint_state_topic}"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
"""
    tmp_bridge_file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp_bridge_file.write(bridge_yaml_content)
    tmp_bridge_file.flush()
    
    bridge_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"aruco_{marker_name}_bridge",
                parameters=[
                    {"config_file": tmp_bridge_file.name},
                    {"use_sim_time": True}
                ],

                output="log"
            )
        ]
    )

    # Config file path
    pkg_share = get_package_share_directory('rovo_gz_description')
    config_path = os.path.join(pkg_share, 'config', 'teleop_joy_config.yaml')

    # Joy node for controller input
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        parameters=[config_path],
        output="screen"
    )

    # Teleop node - publishes to the bot's cmd_vel topic
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        remappings=[("cmd_vel", ros_cmd_vel_topic)],
        parameters=[{"use_sim_time": True}, config_path],
        output="screen"
    )

    actions = [remove_entity_cmd, spawn_node, bridge_node, joy_node, teleop_node]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="1.0",
                              description="X position to spawn the model"),
        DeclareLaunchArgument("y", default_value="1.0",
                              description="Y position to spawn the model"),
        DeclareLaunchArgument("z", default_value="0.01",
                              description="Z position (height) to spawn the model"),
        DeclareLaunchArgument("roll", default_value="0.0",
                               description="Roll angle to spawn the model"),
        DeclareLaunchArgument("pitch", default_value="0.0",
                               description="Pitch angle to spawn the model"),
        DeclareLaunchArgument("yaw", default_value="1.5708",
                              description="Yaw angle to spawn the model"),
        DeclareLaunchArgument("marker_name", default_value="aruco_bot",
                              description="Name of the spawned ArUco bot"),
        DeclareLaunchArgument("world", default_value="empty",
                              description="Name of the Gazebo world (for topic namespacing)"),
        OpaqueFunction(function=_launch_setup),
    ])