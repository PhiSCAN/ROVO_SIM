"""Spawn ArUco marker or ArUco bot into an existing Gazebo Sim world.

This launch file spawns either a static ArUco marker or a mobile ArUco bot
into a running Gazebo simulation. For the mobile ArUco bot, it also sets up
a cmd_vel publisher to make the robot move autonomously.

It assumes a gz server is already running (started by another launch file or the user).

Usage:
  # Spawn static ArUco marker
  ros2 launch rovo_gz_description spawn_aruco.launch.py

  # Spawn mobile ArUco bot with circular movement
  ros2 launch rovo_gz_description spawn_aruco.launch.py model_type:=bot marker_name:=aruco_bot

  # Spawn mobile ArUco bot with custom movement pattern
  ros2 launch rovo_gz_description spawn_aruco.launch.py model_type:=bot marker_name:=aruco_bot movement:=circle radius:=2.0 speed:=0.5
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PKG = "rovo_gz_description"


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PKG)

    # Get launch configurations
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    marker_name = LaunchConfiguration("marker_name").perform(context)
    model_type = LaunchConfiguration("model_type").perform(context)
    world = LaunchConfiguration("world").perform(context)
    movement = LaunchConfiguration("movement").perform(context)
    radius = LaunchConfiguration("radius").perform(context)
    speed = LaunchConfiguration("speed").perform(context)

    # Set GZ_SIM_RESOURCE_PATH to include the models directory
    models_path = os.path.join(os.path.dirname(pkg_share), "models")
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join([
            models_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]),
    )

    # Determine which model to spawn
    if model_type == "bot":
        model_uri = "model://aruco_bot"
    else:
        model_uri = "model://aruco_marker"

    # Spawn the model using ros_gz_sim create
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_aruco",
        arguments=[
            "-name", marker_name,
            "-x", x, "-y", y, "-z", z,
            "-R", roll, "-P", pitch, "-Y", yaw,
        ],
        parameters=[
            {"model": model_uri},
        ],
        output="screen",
    )

    actions = [set_gz_path, spawn_node]

    # For the bot, add a cmd_vel publisher to make it move
    if model_type == "bot":
        # The gazebo sim will have a topic like: /world/{world}/model/{marker_name}/cmd_vel
        # We'll publish to the ROS side via ros_gz_bridge
        ros_cmd_vel_topic = f"/aruco/{marker_name}/cmd_vel"
        gz_cmd_vel_topic = f"world/{world}/model/{marker_name}/cmd_vel"
        
        # Create a bridge config for this specific aruco bot
        import tempfile
        bridge_yaml_content = f"""- ros_topic_name: "{ros_cmd_vel_topic}"
  gz_topic_name: "{gz_cmd_vel_topic}"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
"""
        tmp_bridge_file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        tmp_bridge_file.write(bridge_yaml_content)
        tmp_bridge_file.flush()
        
        # Bridge node
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
        
        # Movement publisher node
        movement_node = TimerAction(
            period=3.0,  # Give bridge time to start
            actions=[
                Node(
                    package="rovo_gz_description",
                    executable="aruco_bot_movement.py",
                    name=f"aruco_{marker_name}_movement",
                    parameters=[
                        {"cmd_vel_topic": ros_cmd_vel_topic},
                        {"movement_pattern": movement},
                        {"radius": float(radius)},
                        {"speed": float(speed)},
                        {"use_sim_time": True}
                    ],
                    output="log"
                )
            ]
        )
        actions.extend([bridge_node, movement_node])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="1.0",
                              description="X position to spawn the model"),
        DeclareLaunchArgument("y", default_value="0.0",
                              description="Y position to spawn the model"),
        DeclareLaunchArgument("z", default_value="0.01",
                              description="Z position (height) to spawn the model"),
        DeclareLaunchArgument("roll", default_value="0.0",
                               description="Roll angle to spawn the model"),
        DeclareLaunchArgument("pitch", default_value="0.0",
                               description="Pitch angle to spawn the model"),
        DeclareLaunchArgument("yaw", default_value="0.0",
                              description="Yaw angle to spawn the model"),
        DeclareLaunchArgument("marker_name", default_value="aruco_marker",
                              description="Name of the spawned entity"),
        DeclareLaunchArgument("model_type", default_value="marker",
                              description="Type of model: 'marker' for static ArUco, 'bot' for mobile robot with ArUco"),
        DeclareLaunchArgument("world", default_value="empty",
                              description="Name of the Gazebo world (for topic namespacing)"),
        DeclareLaunchArgument("movement", default_value="circle",
                              description="Movement pattern: 'circle', 'square', 'figure8', 'backforth', or 'stop'"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Radius for circular movement (meters)"),
        DeclareLaunchArgument("speed", default_value="0.5",
                              description="Movement speed (m/s)"),
        OpaqueFunction(function=_launch_setup),
    ])