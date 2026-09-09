# ArUco Bot Model

A simple differential drive robot with an ArUco marker mounted on top. This model is useful for testing ArUco marker detection with mobile targets in Gazebo simulation.

## Model Structure

- `model.config` - Model metadata
- `model.sdf` - Complete robot model with diff-drive and ArUco marker

## Model Features

- **Base**: Gray rectangular chassis (0.4m x 0.3m x 0.2m)
- **Wheels**: Two black cylinder wheels (radius 0.05m, length 0.08m)
- **ArUco Marker**: White square marker (0.1m x 0.1m x 0.02m) mounted on top
- **Diff-Drive Plugin**: Control via `/cmd_vel` topic
- **Joint States**: Published for wheel joints
- **Frames**: Includes `aruco_marker_frame` for detection reference

## Dimensions

- **Overall size**: 0.4m (length) x 0.3m (width) x 0.31m (height to marker top)
- **Wheel separation**: 0.3m (center to center)
- **Total mass**: ~12kg (10kg base + 2x1kg wheels + 0.01kg marker)

## Usage

To spawn into a running Gazebo simulation:

```xml
<include>
  <uri>model://aruco_bot</uri>
  <name>aruco_bot_1</name>
  <pose>2 0 0 0 0 0</pose>
</include>
```

Or via ros2 launch:
```bash
ros2 launch rovo_gz_description spawn_aruco.launch.py model_type:=bot x:=2.0 y:=0.0 marker_name:=aruco_bot
```

## Control

The robot can be controlled via the `/cmd_vel` topic with `geometry_msgs/Twist` messages:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Frame Tree

```
world
└── aruco_bot
    ├── base_link
    │   ├── left_wheel
    │   │   └── left_wheel_joint
    │   ├── right_wheel
    │   │   └── right_wheel_joint
    │   └── aruco_marker_link
    │       └── aruco_marker_joint
    └── aruco_marker_frame (attached to aruco_marker_link)
```

## Topics

- **cmd_vel** (`geometry_msgs/Twist`) - Velocity commands
- **odom** (`nav_msgs/Odometry`) - Odometry data
- **joint_state** (`sensor_msgs/JointState`) - Joint states