# Warehouse Walking Actors — Design

## Goal

Add animated Gazebo `<actor>` walking people to the warehouse world
(ref: https://gazebosim.org/docs/latest/actors/). Enabled by default, with a
launch argument to turn them off.

## Constraints / Context

- Worlds are plain SDF loaded by name via `gz_args` in
  `src/clearpath_simulator/clearpath_gz/launch/gz_sim.launch.py`. No templating.
- `GZ_SIM_RESOURCE_PATH` already includes the `clearpath_gz/worlds` dir.
- SDF has no native on/off switch for an element, so the toggle lives in launch.
- `warehouse.sdf` already contains *static* people models; those stay untouched.

## Approach: launch arg + spawn node

Actors are defined in a separate SDF and spawned into the running warehouse via a
`ros_gz_sim create` node gated by a launch argument. Keeps `warehouse.sdf` clean,
no world duplication, toggle = run or omit the spawn node.

## Components

### 1. `clearpath_gz/worlds/walking_actors.sdf`

Contains 2 `<actor>` elements (skin + skeleton animation + looped trajectory).

- Skin/animation: Gazebo Fuel `actor` model (`walk.dae`), matching the docs example.
- `<script><loop>true</loop><auto_start>true</auto_start>` with `<trajectory>`
  waypoints forming a closed loop; timing tuned for ~1 m/s so heading follows path.
- `actor_walker_0`: central open area loop, approx
  (-6,0) → (6,0) → (6,6) → (-6,6) → back.
- `actor_walker_1`: north nav-challenge area loop, approx
  (-12,12) → (-4,12) → (-4,20) → (-12,20) → back.
- Waypoint z ≈ 1.0 (actor origin is mid-body per Gazebo actor convention).

Paths chosen to avoid shelf/barrier/table poses present in `warehouse.sdf`.

### 2. `gz_sim.launch.py` changes

- New arg: `DeclareLaunchArgument('actors', default_value='true',
  choices=['true','false'], description='Spawn animated walking actors')`.
- In `gz_launch()`: when `actors == 'true'`, append a `ros_gz_sim` `create` node
  that spawns `walking_actors.sdf` into the world (`-world <world>`,
  `-file <resolved path>`). When `'false'`, omit it.
- Spawn node resolves the world name from the `world` LaunchConfiguration so it
  targets whatever world is loaded (default `warehouse`).

## Data Flow

launch(actors=true) → gz starts warehouse.sdf → create node injects
walking_actors.sdf → SceneBroadcaster animates actors via trajectory script.

## Error Handling / Risks

- Actor spawn into a running world can race world load; use `create` node which
  waits on the create service, so it retries until the world is up.
- If Fuel download fails offline, actors won't appear; sim still runs. Acceptable.
- Actor z offset wrong = feet clip/float; tune after first visual check.

## Testing / Verification

- `world:=warehouse` (default) → 2 people visibly walk their loops.
- `actors:=false` → no walkers, no spawn node, sim otherwise identical.
- Confirm walkers do not spawn inside shelves/barriers.

## Out of Scope

- Collision avoidance between actors and robot (Gazebo actors are non-physical by
  default; they pass through obstacles). Nav costmap treatment not addressed here.
- Per-actor GUI toggle at runtime.
