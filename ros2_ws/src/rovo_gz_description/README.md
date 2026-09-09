# rovo_gz_description

URDF/xacro robot description for the Rovo tracked robot with Gazebo Sim
(Harmonic/Ionic) integration for ROS 2 Rolling.

## Quick Start (Docker)

Launch the full sim stack (Gazebo + RViz + fake pico bridge) inside the
`navigation2:development` container:

```bash
cd /home/godfather/Work/Claude/Rovo_Model/ROVO_SIM
./initialize_simulation.sh
```

Script mounts `ROVO_SIM/` at `/opt/ROVO_SIM`, runs `colcon build`,
sources the overlay, and launches `sim_bringup.launch.py`.

## Run inside an already-running container

```bash
docker exec -it initialize_simulation bash
```

Then inside the container:

```bash
source /opt/ros/rolling/setup.bash && \
source /opt/clearpath_ws/install/setup.bash && \
source /opt/ros2_ws/install/setup.bash && \
ros2 launch rovo_gz_description rovo_clearpath.launch.py
```

With RViz (warehouse world, single copy-paste):

```bash
docker exec -it nav2-rolling bash -c "source /opt/ros/rolling/setup.bash && source /opt/clearpath_ws/install/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 launch rovo_gz_description rovo_clearpath.launch.py world:=warehouse launch_rviz:=true"
```

Other worlds: `pipeline`, `construction`, `office`, `orchard`, `solar_farm`,
`warehouse`, `empty`.

Other launches (same sourcing prefix):

```bash
# Full bringup (Gazebo + RViz + fake pico)
ros2 launch rovo_gz_description sim_bringup.launch.py

# Gazebo Sim + bridge
ros2 launch rovo_gz_description gz_sim.launch.py

# RViz only (no Gazebo)
ros2 launch rovo_gz_description display.launch.py

# Spawn rovo into an existing gz world
ros2 launch rovo_gz_description spawn_rovo_gz.launch.py

# RealSense streams standalone
ros2 launch rovo_gz_description realsense_verify.launch.py
```

## Drive the robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_gcs
```

## Env defaults inside container

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=50
```

## World / model overrides

```bash
WORLD=empty.sdf MODEL=rovo_tracked.urdf.xacro ./initialize_simulation.sh
```

## Layout

```
rovo_gz_description/
├── urdf/
│   ├── rovo.urdf.xacro              # base description
│   ├── rovo_tracked.urdf.xacro      # tracked variant (default)
│   └── rovo_diff_drive.urdf.xacro   # diff-drive variant
├── launch/
│   ├── sim_bringup.launch.py        # full sim bringup
│   ├── display.launch.py            # RViz + joint_state GUI (no Gazebo)
│   ├── gz_sim.launch.py             # Gazebo Sim spawn + bridge
│   ├── spawn_rovo_gz.launch.py      # spawn into existing gz world
│   ├── rovo_clearpath.launch.py     # spawn in clearpath worlds
│   └── realsense_verify.launch.py   # standalone camera check
├── config/
│   ├── bridge.yaml                  # ros_gz_bridge topic map
│   ├── camera_*.yaml                # camera bridges
│   └── *.rviz                       # RViz presets
├── meshes/{visual,collision}/       # 21 parts from body.glb
└── worlds/empty.sdf
```

## Topics

| ROS topic       | Direction   | gz topic                                      |
|-----------------|-------------|-----------------------------------------------|
| `/cmd_vel_gcs`  | pub → gz    | `/cmd_vel_gcs`                                |
| `/odom`         | sub ← gz    | `/odom`                                       |
| `/tf`           | sub ← gz    | `/tf`                                         |
| `/joint_states` | sub ← gz    | `/world/empty/model/rovo/joint_state`         |
| `/imu`          | sub ← gz    | `/imu`                                        |
| `/clock`        | sub ← gz    | `/clock`                                      |

## Rig notes

- **Track sim**: fake diff-drive. 21 GLB parts (body, tracks, wheels,
  sprockets) are visual children of `base_link`. Two invisible cylindrical
  drive wheels at `(±0.547, 0, 0.09)` provide rolling contact via the
  gz-sim DiffDrive system. Separation 1.094 m, radius 0.09 m.
- **base_footprint**: ground projection frame (Nav2 expects this).
- **Collision**: single chassis box (1.312 × 1.464 × 0.728). Tracks and
  road wheels are visual-only for contact stability.
- **Inertia**: box for `base_link` (50 kg), cylinders for drive wheels
  (3 kg). Tune in xacro properties at top.

## Regenerate meshes from body.glb

```bash
cd /home/godfather/Work/Claude/Rovo_Model
./blender-3.3.21-linux-x64/blender --background --python export_meshes.py
```

## Tuning

- Robot sinks: lower spawn `-z` or check `base_link` collision origin
  (currently `0.544`).
- Tracks slip: bump `<mu1>/<mu2>` on `left_wheel`/`right_wheel` gazebo
  blocks (currently 1.2).
- Diff-drive too slow/fast: tweak `<max_linear_acceleration>` or send
  larger `geometry_msgs/Twist` `linear.x`.
- `bridge.yaml` hardcodes world name `empty` in joint_state topic.
  Update if loading a different world.
