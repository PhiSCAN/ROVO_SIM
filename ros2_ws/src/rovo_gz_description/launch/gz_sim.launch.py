"""Spawn Rovo in Gazebo Sim (Harmonic/Ionic) with tracked-vehicle + bridge."""
import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                             DeclareLaunchArgument, ExecuteProcess, TimerAction,
                             OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    pkg = "rovo_gz_description"
    pkg_share = get_package_share_directory(pkg)

    # ── World resolution ──────────────────────────────────────────────────────
    world_arg = LaunchConfiguration("world").perform(context)

    if os.path.isabs(world_arg) and os.path.isfile(world_arg):
        world_path = world_arg
    else:
        candidates = [os.path.join(pkg_share, "worlds", world_arg)]
        try:
            clearpath_share = get_package_share_directory("clearpath_gz")
            candidates.append(os.path.join(clearpath_share, "worlds", world_arg))
        except Exception:
            pass
        world_path = next((p for p in candidates if os.path.isfile(p)), None)
        if world_path is None:
            raise RuntimeError(f"World '{world_arg}' not found. Searched: {candidates}")

    # World name = SDF <world name="..."> = filename without .sdf
    world_name = os.path.splitext(os.path.basename(world_path))[0]

    # ── Model / xacro ─────────────────────────────────────────────────────────
    model_arg = LaunchConfiguration("model").perform(context)
    xacro_path = os.path.join(pkg_share, "urdf", model_arg)

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]), value_type=str
        ),
        "use_sim_time": True,
    }

    # ── Bridge config: inject correct world name for joint_state topic ─────────
    bridge_template = os.path.join(pkg_share, "config", "bridge.yaml")
    with open(bridge_template) as f:
        raw = f.read()
    bridge_content = raw.replace("world/empty/model/rovo",
                                 f"world/{world_name}/model/rovo")
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp.write(bridge_content)
    tmp.flush()
    bridge_yaml = tmp.name

    # ── GZ_SIM_RESOURCE_PATH ──────────────────────────────────────────────────
    resource_paths = [os.path.dirname(pkg_share)]
    try:
        clearpath_share = get_package_share_directory("clearpath_gz")
        resource_paths += [os.path.dirname(clearpath_share), clearpath_share,
                           os.path.join(clearpath_share, "meshes")]
    except Exception:
        pass
    resource_paths.append(os.environ.get("GZ_SIM_RESOURCE_PATH", ""))

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
        parameters=[robot_description],
        output="screen",
    )

    # Delay spawn so Gazebo has time to initialise the world and register services
    spawn = TimerAction(
        period=8.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "rovo", "-topic", "robot_description", "-z", "0.0"],
            output="screen",
        )],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
        output="screen",
    )

    # Dedicated camera bridge — mirrors realsense_verify approach exactly.
    # Delayed past spawn+sensor-init to avoid partial subscription on startup.
    camera_bridge_yaml = os.path.join(pkg_share, "config", "camera_robot_bridge.yaml")
    camera_bridge = TimerAction(
        period=12.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            parameters=[{"config_file": camera_bridge_yaml, "use_sim_time": True}],
            output="screen",
        )],
    )

    zero_cmd = TimerAction(
        period=14.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "topic", "pub", "--once", "/cmd_vel_gcs",
                 "geometry_msgs/msg/Twist",
                 "{linear: {x: 0.0}, angular: {x: 0.0}}"],
            output="log",
        )],
    )

    # Convert Gazebo 32FC1 depth (metres) → 16UC1 (mm) to match RealSense driver.
    # Publishes on /camera/camera/depth/image_rect_raw_uint16.
    depth_converter = Node(
        package="rovo_gz_description",
        executable="depth_float32_to_uint16.py",
        name="depth_float32_to_uint16",
        parameters=[{"use_sim_time": True}],
        output="log",
    )

    rviz_config = os.path.join(pkg_share, "config", "robot_camera_verify.rviz")
    rviz = TimerAction(
        period=14.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )],
    )

    return [set_gz_path, gz_sim, rsp, spawn, bridge, camera_bridge, zero_cmd, depth_converter, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="rovo_tracked.urdf.xacro",
                              description="URDF xacro filename in urdf/"),
        DeclareLaunchArgument("world", default_value="empty.sdf",
                              description="World .sdf filename or absolute path"),
        DeclareLaunchArgument("launch_rviz", default_value="false",
                              description="Launch RViz with camera verification config"),
        OpaqueFunction(function=_launch_setup),
    ])
