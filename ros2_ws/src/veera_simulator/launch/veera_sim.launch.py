"""Veera Robot Simulation Launch File - Full Simulation Bringup.

Launches Gazebo Sim with Veera robot, ROS-GZ bridge, state publisher,
Zenoh router, Fake PICO, and optional actors.

Replaces clearpath_simulator functionality while using local assets.

Usage:
  ros2 launch veera_simulator veera_sim.launch.py world:=warehouse model:=tracked

Available worlds: empty, construction, office, orchard, pipeline, solar_farm, warehouse
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# World-specific spawn poses to avoid collisions
_WORLD_SPAWN_DEFAULTS = {
    "empty":       ("0.0", "0.0", "0.0"),
    "pipeline":    ("0.0", "0.0", "0.0"),
    "warehouse":   ("0.0", "4.0", "0.0"),  # Origin blocked by obstacles
    "office":      ("0.0", "0.0", "0.0"),
    "construction": ("0.0", "0.0", "0.0"),
    "orchard":     ("0.0", "0.0", "0.0"),
    "solar_farm":  ("0.0", "0.0", "0.0"),
}


def _launch_setup(context, *args, **kwargs):
    """Main launch setup function."""
    # Get launch configurations
    world_name = LaunchConfiguration("world").perform(context)
    model_name = LaunchConfiguration("model").perform(context)
    use_bridge = LaunchConfiguration("use_bridge").perform(context) == "true"
    use_actors = LaunchConfiguration("actors").perform(context) == "true"
    fake_pico_port = LaunchConfiguration("fake_pico_port").perform(context)

    # Build paths
    veera_share = get_package_share_directory("veera_simulator")
    rovo_share = get_package_share_directory("rovo_gz_description")
    
    world_path = os.path.join(veera_share, "worlds", f"{world_name}.sdf")
    model_path = os.path.join(rovo_share, "urdf", f"rovo_{model_name}.urdf.xacro")
    
    world_base = world_name

    # Robot description parameter
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", model_path]), value_type=str
        ),
        "use_sim_time": True,
    }

    # Apply per-world spawn defaults
    cli_x = LaunchConfiguration("x").perform(context)
    cli_y = LaunchConfiguration("y").perform(context)
    cli_z = LaunchConfiguration("z").perform(context)
    dx, dy, dz = _WORLD_SPAWN_DEFAULTS.get(world_base, ("0.0", "0.0", "0.0"))
    spawn_x = cli_x if cli_x not in ("0.0", "0") else dx
    spawn_y = cli_y if cli_y not in ("0.0", "0") else dy
    spawn_z = cli_z if cli_z not in ("0.0", "0") else dz

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # # Zenoh router (for DDS communication)
    # zenoh_router = Node(
    #     package="rmw_zenoh_cpp",
    #     executable="rmw_zenohd",
    #     name="rmw_zenohd",
    #     output="screen",
    # )

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Spawn robot entity (delayed to let Gazebo initialize)
    spawn_robot = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_veera",
                arguments=[
                    "-name", "veera",
                    "-topic", "robot_description",
                    "-x", spawn_x,
                    "-y", spawn_y,
                    "-z", spawn_z,
                    "-Y", LaunchConfiguration("yaw"),
                ],
                output="screen",
            )
        ],
    )

    # Fake PICO (for teleoperation interface)
    fake_pico = []
    try:
        fake_pico_share = get_package_share_directory("rovo_fake_pico")
        fake_pico = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fake_pico_share, "launch", "fake_pico.launch.py")
            ),
            launch_arguments={
                "listen_port": fake_pico_port,
                "output_topic": "/cmd_vel_gcs",
            }.items(),
        )]
    except Exception as e:
        LogInfo(msg=f"[veera_sim] rovo_fake_pico not available: {e}")

    # Animated walking actors (only for warehouse world)
    actor_nodes = []
    if use_actors and world_base == "warehouse":
        for f in ("walking_actor_0.sdf", "walking_actor_1.sdf"):
            actor_path = os.path.join(veera_share, "worlds", f)
            if os.path.isfile(actor_path):
                actor_nodes.append(Node(
                    package="ros_gz_sim",
                    executable="create",
                    name=f"spawn_{os.path.splitext(f)[0]}",
                    arguments=["-world", world_base, "-file", actor_path],
                    output="screen",
                ))
    spawn_actors = TimerAction(period=5.0, actions=actor_nodes) if actor_nodes else []

    actions = [
        # zenoh_router,
        gz_sim,
        rsp,
        spawn_robot,
    ]
    
    actions.extend(fake_pico)
    if spawn_actors:
        actions.append(spawn_actors)

    # ROS-GZ Bridge (optional)
    if use_bridge:
        bridge_yaml = os.path.join(veera_share, "config", "bridge.yaml")
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[
                {"config_file": bridge_yaml},
                {"use_sim_time": True},
            ],
            output="screen",
        )
        actions.append(bridge)

    return actions


def generate_launch_description():
    """Generate launch description with configurable arguments."""
    return LaunchDescription(
        [
            # World selection
            DeclareLaunchArgument(
                "world",
                default_value="empty",
                description="World name. Available: empty, construction, office, "
                            "orchard, pipeline, solar_farm, warehouse",
            ),
            # Robot model selection
            DeclareLaunchArgument(
                "model",
                default_value="tracked",
                description="Robot variant: tracked | diff_drive | default"
            ),
            # Spawn pose
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            # Fake PICO
            DeclareLaunchArgument(
                "fake_pico_port", default_value="8887",
                description="Port for rovo_fake_pico to listen on"
            ),
            # Optional: Enable ROS-GZ bridge
            DeclareLaunchArgument(
                "use_bridge", default_value="true",
                description="Enable ROS-GZ bridge for sensor topics"
            ),
            # Optional: Spawn actors
            DeclareLaunchArgument(
                "actors", default_value="true", choices=["true", "false"],
                description="Spawn animated walking actors (warehouse only)"
            ),
            # Opaque function for delayed evaluation
            OpaqueFunction(function=_launch_setup),
        ]
    )
