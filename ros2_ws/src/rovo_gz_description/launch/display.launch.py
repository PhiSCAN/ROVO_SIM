"""RViz display launch: robot_state_publisher + joint_state_publisher_gui + RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("rovo_gz_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "rovo.urdf.xacro"])
    rviz_path = PathJoinSubstitution([pkg, "config", "rovo.rviz"])

    robot_description = {"robot_description": ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str)}

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=None,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
