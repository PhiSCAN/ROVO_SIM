"""Modular Rovo spawn launch — drop into any already-running Gazebo Sim world.

Does NOT start `gz sim`. Assumes a gz server is running (started by another
launch file or the user). Brings up:
  - robot_state_publisher (publishes /robot_description + /tf)
  - ros_gz_sim/create   (spawns the Rovo model into the running world)
  - ros_gz_bridge       (cmd_vel / raw_odom / joint_states / imu / clock)
  - zero cmd_vel pulse  (avoids stale DiffDrive command across re-spawns)

Launch args:
  x, y, z, yaw      initial pose (default 0/0/0/0)
  robot_name        name of the spawned entity (default 'rovo')
  world             world name used to build the gz joint_state topic path
                    (default 'empty' — must match the world actually running)
  use_sim_time      default true

Example use from another launch file:

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.substitutions import FindPackageShare
    from launch.substitutions import PathJoinSubstitution

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rovo_gz_description'),
            'launch', 'spawn_rovo_gz.launch.py'
        ])),
        launch_arguments={
            'x': '2.0', 'y': '0.0', 'yaw': '0.0',
            'world': 'my_world',
        }.items(),
    )
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PKG = "rovo_gz_description"


def _build_bridge_yaml(world_name: str) -> str:
    """Generate a per-launch bridge YAML with the world name baked into the
    gz joint_state topic. ros_gz_bridge needs the literal topic string."""
    yaml = f"""\
- ros_topic_name: "cmd_vel_gcs"
  gz_topic_name: "cmd_vel_gcs"
  ros_type_name: "geometry_msgs/msg/TwistStamped"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "raw_odom"
  gz_topic_name: "odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "joint_states"
  gz_topic_name: "world/{world_name}/model/rovo/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data2"
  gz_topic_name: "imu/data2"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# === Velodyne VLP-16 ===
- ros_topic_name: "/velodyne_points_raw"
  gz_topic_name: "velodyne_points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

# === RealSense D435i RGBD ===
- ros_topic_name: "/camera/camera/color/image_raw"
  gz_topic_name: "camera/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera/depth/image_rect_raw"
  gz_topic_name: "camera/camera/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera/depth/color/points"
  gz_topic_name: "camera/camera/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera/color/camera_info"
  gz_topic_name: "camera/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera/imu"
  gz_topic_name: "camera/camera/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
"""
    out = f"/tmp/rovo_bridge_{world_name}.yaml"
    with open(out, "w") as f:
        f.write(yaml)
    return out


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PKG)

    world = LaunchConfiguration("world").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    model = LaunchConfiguration("model").perform(context)

    # Map model arg to xacro file. Accepts: diff_drive, tracked.
    # Empty / "default" / unknown → rovo.urdf.xacro (legacy single file).
    xacro_map = {
        "diff_drive": "rovo_diff_drive.urdf.xacro",
        "tracked":    "rovo_tracked.urdf.xacro",
        "default":    "rovo.urdf.xacro",
    }
    xacro_file = xacro_map.get(model, "rovo.urdf.xacro")
    xacro_path = os.path.join(pkg_share, "urdf", xacro_file)

    bridge_yaml = _build_bridge_yaml(world)

    # Make package://rovo_gz_description/... resolvable to gz mesh loader
    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join([
            os.path.dirname(pkg_share),
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ]),
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]), value_type=str,
        ),
        "use_sim_time": use_sim_time.lower() in ("true", "1"),
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        # /tf remapped: Gazebo bridge already publishes dynamic TF.
        # /tf_static kept so rsp broadcasts URDF fixed joints
        # (base_footprint -> base_link, sensors, tracks).
        remappings=[
            ("/tf", "/rsp_tf_unused"),
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "robot_description",
            "-world", world,
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_yaml,
            "use_sim_time": use_sim_time.lower() in ("true", "1"),
        }],
        output="screen",
    )

    # Convert gz 32FC1 depth (metres) -> 16UC1 (mm) to match RealSense driver
    # and feed RViz Image displays expecting _uint16.
    depth_converter = Node(
        package="rovo_gz_description",
        executable="depth_float32_to_uint16.py",
        name="depth_float32_to_uint16",
        parameters=[{"use_sim_time": use_sim_time.lower() in ("true", "1")}],
        output="log",
    )

    velodyne_time_adapter = Node(
        package="rovo_gz_description",
        executable="velodyne_time_adapter.py",
        name="velodyne_time_adapter",
        parameters=[{"use_sim_time": use_sim_time.lower() in ("true", "1")}],
        output="log",
    )

    zero_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once", "/cmd_vel_gcs",
                    "geometry_msgs/msg/TwistStamped",
                    "{header: {frame_id: base_link}, twist: "
                    "{linear: {x: 0.0, y: 0.0, z: 0.0}, "
                    "angular: {x: 0.0, y: 0.0, z: 0.0}}}",
                ],
                output="log",
            ),
        ],
    )

    return [set_gz_path, rsp, spawn, bridge, depth_converter,
            velodyne_time_adapter, zero_cmd]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.15"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_name", default_value="rovo"),
        DeclareLaunchArgument(
            "world", default_value="empty",
            description="Running world name (used in gz joint_state topic)."
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value=os.environ.get("USE_SIM_TIME", "true").lower()
        ),
        DeclareLaunchArgument(
            "model", default_value="tracked",
            description="Robot variant: 'tracked' (TrackedVehicle plugin) | "
                        "'diff_drive' (cylinder wheels) | 'default' "
                        "(legacy rovo.urdf.xacro)."
        ),
        OpaqueFunction(function=_launch_setup),
    ])
