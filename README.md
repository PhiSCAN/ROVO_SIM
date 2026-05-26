# ROVO_SIM

Rovo simulation workspace. One script clones submodules, builds inside
`navigation2:development` container, launches Zenoh + Gazebo + RViz +
`rovo_fake_pico` UDP bridge.

## Setup

```bash
git clone --recurse-submodules git@github.com:phicode-robotics/ROVO_SIM.git
cd ROVO_SIM
./initialize_simulation.sh
```

Default: empty world + RViz + fake_pico.

## Worlds

`empty`, `warehouse`, `pipeline`, `office`, `orchard`, `solar_farm`,
`construction`.

## Run with a specific world (e.g. warehouse)

In another terminal, while container runs:

```bash
docker exec -it initialize_simulation bash -c "source /opt/ros/rolling/setup.bash && source /opt/ROVO_SIM/install/setup.bash && ros2 launch rovo_gz_description rovo_clearpath.launch.py world:=warehouse launch_rviz:=true"
```

Swap `world:=warehouse` for any world above.

## Drive

```bash
docker exec -it initialize_simulation bash -c "source /opt/ROVO_SIM/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_gcs"
```

## Enable realsense driver

```bash
rm src/realsense-ros/COLCON_IGNORE
```

## Update submodules

```bash
git submodule update --remote --merge
```
