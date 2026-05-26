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

Default world: `empty`.

## Worlds

```bash
./initialize_simulation.sh empty
./initialize_simulation.sh warehouse
./initialize_simulation.sh pipeline
./initialize_simulation.sh office
./initialize_simulation.sh orchard
./initialize_simulation.sh solar_farm
./initialize_simulation.sh construction
```

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
