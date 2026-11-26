# ROVO_SIM

Rovo simulation workspace. One script clones submodules, builds inside
`navigation2:development` container, launches DDS + Gazebo + RViz +
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

## LAZ terrain

`Point cloud-001.laz` is a 272-million-point LAS 1.2 cloud, and `dem.tif` is a
0.5 m georeferenced DTM; both use WGS 84 / UTM zone 43N. The terrain world
combines class-2 LAZ ground returns with the DTM: the DEM supplies a continuous
base and fills missing returns, while the point cloud adds local ground detail.
The active mesh uses the DTM's native 0.5 m cell size. The previous 1 m mesh is
kept alongside it as `lidar_terrain_1m.stl` for comparison. The active mesh is
`ros2_ws/src/veera_simulator/meshes/terrain/lidar_terrain.stl`.

To regenerate it, install `laspy[lazrs]`, `numpy`, `rasterio`, `scipy`, and
`pyproj`, then run:

```bash
python3 tools/convert_laz_to_terrain.py "Point cloud-001.laz" dem.tif \
  ros2_ws/src/veera_simulator/meshes/terrain/lidar_terrain.stl --cell-size 0.5
```

Build the workspace and launch the terrain world with:

```bash
./colcon_build.sh
./initialize_simulation.sh terrain
```

The mesh covers the combined valid DTM/ground-return footprint and is centered
on the DTM bounds. The terrain world spawns the robot at
`(125.446, 282.865, -11.3239)`, matching the current verified pose on a fully
meshed patch. The
converter reports any DTM/cloud vertical offset and how much it fills. Use
smaller grid spacing for more detail (at the cost of a larger/slower mesh).

## Worlds

```bash
./initialize_simulation.sh empty
./initialize_simulation.sh warehouse
./initialize_simulation.sh pipeline
./initialize_simulation.sh office
./initialize_simulation.sh orchard
./initialize_simulation.sh solar_farm
./initialize_simulation.sh construction
./initialize_simulation.sh terrain
```

## Drive

In a separate terminal:

```bash
./initialize_teleop.sh
```

Execs into the running container, runs `teleop_twist_keyboard` publishing
TwistStamped on `/gcs_cmd_vel`.

## Enable realsense driver

```bash
rm src/realsense-ros/COLCON_IGNORE
```

## Update submodules

```bash
git submodule update --remote --merge
```
