# ArUco Marker Model

This is a simple Gazebo/Ignition Simulation model for ArUco markers.

## Model Structure

- `model.config` - Model metadata
- `model.sdf` - Model definition (SDF format)
- `materials/` - Directory for custom materials and textures

## Model Details

- **Size**: 10cm x 10cm x 2mm (standard ArUco marker size)
- **Mass**: 0.01 kg (very light, suitable for markers on walls/floors)
- **Color**: White base (ArUco markers are typically printed on white background)
- **Frame**: Includes a `marker_frame` that represents the coordinate system at the marker's center

## Usage in Gazebo Worlds

To use this model in your Gazebo world file, add:

```xml
<include>
  <uri>model://aruco_marker</uri>
  <name>aruco_marker_1</name>
  <pose>1 0 0.01 0 0 0</pose> <!-- Position where you want the marker -->
</include>
```

## Adding to Gazebo Model Path

To make this model available in Gazebo, add the models directory to your GAZEBO_MODEL_PATH:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/dafodilrat/Documents/phicode/ROVO_SIM/models
```

Or add it to your `.bashrc` or workspace setup.

## Customizing the Marker

To create different ArUco marker IDs, you have several options:

1. **Different SDF files**: Create separate directories for different marker IDs (e.g., `aruco_marker_0`, `aruco_marker_1`, etc.)

2. **Custom Materials**: Add specific ArUco marker images to the `materials/` folder and reference them in the SDF with:

```xml
<material>
  <script>
    <uri>file://materials/aruco_patternmaterial</uri>
  </script>
</material>
```

3. **Programmatic Generation**: Use a script to generate SDF files with different marker IDs dynamically.

## ArUco Marker Detection

For ArUco marker detection in simulation, you'll need:
1. A camera sensor in your robot
2. An ArUco detection node (like `aruco_ros` or `cv_aruco`)
3. The camera properly configured to see the marker

The marker frame (`marker_frame`) can be used as a reference pose for detection algorithms.