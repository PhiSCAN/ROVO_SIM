# ROVO_SIM

Rovo simulation workspace. One script clones submodules, builds the
workspace inside the `navigation2:development` container, and launches
Zenoh router + Gazebo + RViz + `rovo_fake_pico` UDP bridge.

## Layout

```
ROVO_SIM/
├── initialize_simulation.sh
└── src/
    ├── rovo_gz_description     # submodule (phicode-robotics)
    ├── clearpath_simulator     # submodule (clearpath worlds + meshes)
    ├── realsense-ros           # submodule (COLCON_IGNOREd by default)
    └── rovo_fake_pico          # vendored (UDP -> /cmd_vel_gcs)
```

## Quick start

```bash
git clone --recurse-submodules git@github.com:phicode-robotics/ROVO_SIM.git
cd ROVO_SIM
./initialize_simulation.sh
```

Script will:
1. `git submodule update --init --recursive`
2. `touch src/realsense-ros/COLCON_IGNORE`
3. `docker run navigation2:development` with workspace mounted at
   `/opt/ROVO_SIM`
4. `colcon build --symlink-install` + source
5. `ros2 launch rovo_gz_description sim_bringup.launch.py`
   (Zenoh router + gz_sim + RViz + fake_pico)

## Worlds / models

```bash
WORLD=empty.sdf MODEL=rovo_tracked.urdf.xacro ./initialize_simulation.sh
```

For clearpath worlds (warehouse, pipeline, office, orchard, solar_farm,
construction), exec into the running container and run:

```bash
docker exec -it initialize_simulation bash -c "source /opt/ros/rolling/setup.bash && source /opt/ROVO_SIM/install/setup.bash && ros2 launch rovo_gz_description rovo_clearpath.launch.py world:=warehouse launch_rviz:=true"
```

## Drive

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_gcs
```

## Enable realsense driver

Remove the COLCON_IGNORE marker and rebuild:

```bash
rm src/realsense-ros/COLCON_IGNORE
```

## Update submodules

```bash
git submodule update --remote --merge
```
