# ROVO_SIM

Rovo simulation workspace. One script clones submodules, builds inside
`navigation2:development` container, launches Zenoh + Gazebo + RViz +
`rovo_fake_pico` UDP bridge.

## Setup

```bash
git clone --recurse-submodules git@github.com:phicode-robotics/ROVO_SIM.git
cd ROVO_SIM
./colcon_build.sh
./initialize_simulation.sh
```

`colcon_build.sh` runs `rosdep update`, `rosdep install`, and `colcon build`
inside the `navigation2:development` container, producing `build/`,
`install/`, and `log/` in the repo root. Run it once after cloning.

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

In a separate terminal:

```bash
./initialize_teleop.sh
```

Execs into the running container, runs `teleop_twist_keyboard` publishing
TwistStamped on `/cmd_vel_gcs`.

## Enable realsense driver

```bash
rm src/realsense-ros/COLCON_IGNORE
```

## Update submodules

```bash
git submodule update --remote --merge
```
