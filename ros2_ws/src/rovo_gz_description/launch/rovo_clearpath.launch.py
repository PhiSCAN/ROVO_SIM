"""Launch Rovo inside a clearpath_gz world.

Bypasses clearpath_gz/simulation.launch.py (which requires robot.yaml setup)
and instead loads the world SDF directly, then spawns rovo into it.

Brings up:
  - gz_sim with <clearpath_gz>/worlds/<world>.sdf
  - rovo spawn + bridge (via spawn_rovo_gz.launch.py)

Launch args:
  world       clearpath_gz world name (default 'pipeline').
              Resolves to <clearpath_gz share>/worlds/<world>.sdf.
              Available worlds: construction, office, orchard, pipeline,
              solar_farm, warehouse.
  x, y, z, yaw   spawn pose for rovo

Requires:
  - clearpath_config + clearpath_gz built and sourced
  - Source order:
      source /opt/ros/rolling/setup.bash
      source /opt/clearpath_ws/install/setup.bash
      source /opt/ros2_ws/install/setup.bash

Usage:
  ros2 launch rovo_gz_description rovo_clearpath.launch.py world:=pipeline
  ros2 launch rovo_gz_description rovo_clearpath.launch.py world:=warehouse x:=2.0

Per-world sane spawn poses (origin (0,0,0) usually falls off the map):
  pipeline      x:=0  y:=8   z:=0.5   (next to the base_station platform)
  warehouse     x:=0  y:=4   z:=0.0   (free aisle; origin blocked by shelf+person)
  construction  x:=0  y:=0   z:=0.3
  office        x:=0  y:=0   z:=0.1
  orchard       x:=0  y:=0   z:=0.3
  solar_farm    x:=0  y:=0   z:=0.3
Inspect <clearpath_gz>/worlds/<world>.sdf for the exact terrain layout.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_WORLD_SPAWN_DEFAULTS = {
    # World name → (x, y, z).
    # All worlds spawn at z=0.0 (same as empty) to eliminate drop-bounce
    # glitch. Pipeline.sdf is subsea — no ground at any (x,y), robot will
    # fall regardless; spawn at origin and accept that limitation.
    "empty":       ("0.0", "0.0", "0.0"),
    "pipeline":    ("0.0", "0.0", "0.0"),
    # Origin (0,0) sits between shelf_7 (0.4,-2) and a standing person (1,-1),
    # which fills collision_monitor's PolygonStop zone and freezes the robot.
    # (0,4) is open: nearest obstacle ~5m, clear of all stop/slow polygons.
    "warehouse":   ("0.0", "4.0", "0.0"),
    "office":      ("0.0", "0.0", "0.0"),
    "construction": ("0.0", "0.0", "0.0"),
    "orchard":     ("0.0", "0.0", "0.0"),
    "solar_farm":  ("0.0", "0.0", "0.0"),
}


def _resolve_clearpath_world(context):
    world = LaunchConfiguration("world").perform(context)
    rovo_share = get_package_share_directory("rovo_gz_description")

    # Special case: world=='empty' uses our own empty.sdf — no clearpath
    # dependency, fastest for validating URDF + plugins.
    if world == "empty":
        world_path = os.path.join(rovo_share, "worlds", "empty.sdf")
        clearpath_share = None
        if not os.path.isfile(world_path):
            return [LogInfo(msg=f"[rovo_clearpath] empty.sdf missing at "
                                f"{world_path}")]
    else:
        try:
            clearpath_share = get_package_share_directory("clearpath_gz")
        except Exception as e:
            return [LogInfo(msg=f"[rovo_clearpath] ERROR: clearpath_gz not "
                                f"found. Build + source /opt/clearpath_ws "
                                f"first. ({e})")]
        world_path = os.path.join(clearpath_share, "worlds", f"{world}.sdf")
        if not os.path.isfile(world_path):
            avail = sorted(os.listdir(os.path.join(clearpath_share, "worlds")))
            return [LogInfo(msg=f"[rovo_clearpath] World '{world}.sdf' not "
                                f"found. Available: empty, {avail}")]

    # Extend GZ_SIM_RESOURCE_PATH for mesh URI resolution. Rovo's share
    # parent always; clearpath_gz dirs only when not using empty world.
    extra_paths = [
        os.path.dirname(rovo_share),                 # parent of rovo_gz_description/
    ]
    if clearpath_share is not None:
        extra_paths += [
            os.path.dirname(clearpath_share),
            os.path.join(clearpath_share, "worlds"),
            os.path.join(clearpath_share, "meshes"),
            os.path.join(clearpath_share, "geotif"),
        ]
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(extra_paths + [
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]),
    )

    # Start gz with the chosen world. Use ros_gz_sim's gz_sim.launch.py for
    # proper /clock + lifecycle integration.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"),
            "launch", "gz_sim.launch.py",
        )),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # Apply per-world spawn defaults UNLESS user overrode args explicitly.
    # User-passed args take precedence; defaults only fill the 0.0 fallback.
    cli_x = LaunchConfiguration("x").perform(context)
    cli_y = LaunchConfiguration("y").perform(context)
    cli_z = LaunchConfiguration("z").perform(context)
    dx, dy, dz = _WORLD_SPAWN_DEFAULTS.get(world, ("0.0", "0.0", "0.15"))
    spawn_x = cli_x if cli_x not in ("0.0", "0") else dx
    spawn_y = cli_y if cli_y not in ("0.0", "0") else dy
    spawn_z = cli_z if cli_z not in ("0.0", "0") else dz

    # Spawn rovo after short delay so world is up before create call.
    # With GPU EGL enabled, worlds load in <3s. If create node loops on
    # "Requesting list of world names", bump back to 10-15s.
    spawn_rovo = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare("rovo_gz_description"),
                    "launch", "spawn_rovo_gz.launch.py",
                ])),
                launch_arguments={
                    "x": spawn_x,
                    "y": spawn_y,
                    "z": spawn_z,
                    "yaw": LaunchConfiguration("yaw"),
                    "world": world,
                    "model": LaunchConfiguration("model"),
                }.items(),
            ),
        ],
    )

    # Animated walking actors (default on for warehouse). Spawned into the
    # running world after a short delay so the world is up before the create
    # call. Toggle with actors:=false. Paths in the actor SDFs are tuned for
    # the warehouse layout, so only spawn there.
    actors_on = LaunchConfiguration("actors").perform(context) == "true"
    actor_nodes = []
    if actors_on and world == "warehouse":
        for f in ("walking_actor_0.sdf", "walking_actor_1.sdf"):
            actor_path = os.path.join(rovo_share, "worlds", f)
            if not os.path.isfile(actor_path):
                actor_nodes.append(LogInfo(
                    msg=f"[rovo_clearpath] actor file missing: {actor_path}"))
                continue
            actor_nodes.append(Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{os.path.splitext(f)[0]}",
                arguments=["-world", world, "-file", actor_path],
                output="screen",
            ))
    spawn_actors = TimerAction(period=5.0, actions=actor_nodes)

    rviz_config = os.path.join(rovo_share, "config", "robot_camera_verify.rviz")
    rviz = TimerAction(
        period=6.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )],
    )

    return [
        LogInfo(msg=f"[rovo_clearpath] using world: {world_path}"),
        LogInfo(msg=f"[rovo_clearpath] spawn pose: x={spawn_x} y={spawn_y} z={spawn_z}"),
        set_gz_path,
        gz_sim,
        spawn_rovo,
        spawn_actors,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="empty",
            description="World name. 'empty' uses rovo_gz_description's own "
                        "empty.sdf. Other names resolve to "
                        "<clearpath_gz share>/worlds/<world>.sdf."
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "model", default_value="tracked",
            description="Robot variant: tracked | diff_drive | default"
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false",
            description="Launch RViz with rovo.rviz config"
        ),
        DeclareLaunchArgument(
            "actors", default_value="true", choices=["true", "false"],
            description="Spawn animated walking actors (warehouse only)"
        ),
        OpaqueFunction(function=_resolve_clearpath_world),
    ])
