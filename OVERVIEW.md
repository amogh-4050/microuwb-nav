# MicroUWB-Nav — Project Overview

Sub-50g indoor quadcopter simulation using UWB positioning. Proof-of-concept
for a physical nRF52840 + DW3000 drone. This doc is for teammates joining
mid-project and covers what's built, what's next, and how to run it.

---

## What it is

A simulation-first pipeline for a micro quadcopter that navigates indoors
using Ultra-Wideband two-way ranging (TWR) to 5 fixed anchors. The system
measures distances tag-to-anchor, trilaterates a 3D position, smooths with
a Kalman filter, and feeds that into a flight controller.

**Key design choice:** we precompute UWB channel impulse responses offline
with Sionna RT ray tracing, store them in an HDF5 lookup table, and
interpolate at runtime. This gives us physics-accurate multipath behavior
without a ray tracer in the control loop.

---

## Stack

- **ROS2 Humble** on Ubuntu 22.04
- **Gazebo Classic 11** for physics + visualization
- **Sionna RT 1.2.1** for offline UWB channel simulation (conda env)
- **Mitsuba 3** as Sionna's ray-tracing backend
- **Python 3.10** for all ROS nodes and Sionna scripts
- Future hardware: nRF52840 (Zephyr RTOS), DW3000 UWB, LSM6DS3 IMU

---

## Architecture

```
 Offline (conda / sionna_env, one-shot):
 ┌──────────────────────────────────────────────┐
 │ Sionna RT 1.2.1                              │
 │   ↓ ray-trace 386k grid points × 5 anchors   │
 │ HDF5 CIR lookup table (6.6 MB)               │
 └──────────────────────────────────────────────┘
                    │
 Runtime (ROS2 Humble):
 ┌──────────────────▼───────────────────────────┐
 │ uwb_range_simulator — HDF5 interpolator      │
 │   publishes /microuwb/ranges @ 10 Hz         │
 │                    │                         │
 │ trilateration_node — LM nonlinear LSQ        │
 │   publishes /microuwb/position_estimate      │
 │                    │                         │
 │ kalman_filter — 6D constant-velocity KF      │
 │   publishes /microuwb/position_filtered      │
 │                    │                         │
 │ flight_controller — cascaded PID             │
 │   publishes motor commands                   │
 │                    │                         │
 │ Gazebo drone model (URDF + plugins)          │
 └──────────────────────────────────────────────┘
```

Interface contracts (stable across implementation changes):
- `/microuwb/anchors` — latched AnchorArray, 5 anchor positions
- `/microuwb/ranges` — UWBRangeArray at 10 Hz
- `/microuwb/position_estimate` — PoseStamped at 10 Hz
- `/microuwb/position_filtered` — PoseStamped at 10 Hz

---

## Room + anchor geometry

- Room: 5m × 4m × 3m, concrete walls/floor/ceiling
- 3 furniture pieces (bookshelf, cafe table, + 1 more) as NLOS obstacles,
  treated as `itu_wood` in Sionna
- 5 anchors (3 ceiling + 2 floor for good 3D GDOP):

| ID | Position (x,y,z) | Role |
|----|------------------|------|
| a0 | (0.1, 0.1, 2.7) | ceiling corner |
| a1 | (4.9, 0.1, 2.7) | ceiling corner |
| a2 | (2.5, 3.9, 2.7) | ceiling mid-long-wall |
| a3 | (0.1, 2.0, 0.3) | floor mid-short-wall |
| a4 | (4.9, 2.0, 0.3) | floor mid-short-wall |

Source of truth: `src/microuwb_bringup/config/anchors.yaml`.

---

## Repo layout

```
microuwb-nav/
├── src/                               ROS2 packages
│   ├── microuwb_msgs/                 UWBRange, UWBRangeArray, Anchor, AnchorArray
│   ├── microuwb_gazebo/               Room world (xacro), anchor models, launch
│   ├── microuwb_uwb/                  Range sim, trilateration, Kalman filter, gt_relay
│   ├── microuwb_control/              flight_controller (cascaded PID), respawn_guard
│   ├── microuwb_navigation/           (empty — waypoints coming)
│   ├── microuwb_description/          Drone URDF + Xacro, mesh models
│   ├── microuwb_bringup/              Launch files + YAML configs
│   └── microuwb_rviz/                 (empty — RViz configs coming)
├── sionna_precompute/                 Offline CIR pipeline (conda env)
│   ├── scene/room.xml                 Mitsuba 3 scene
│   ├── scene/meshes/                  PLY meshes for walls + furniture
│   ├── notebooks/                     Jupyter sanity + viz notebooks
│   ├── scripts/precompute_cir_table.py  Grid sweep → HDF5
│   ├── scripts/validate_table.py      HDF5 validation + plots
│   ├── scripts/fix_variance_los_heuristic.py  Post-processing
│   └── data/room_cir_table_full.h5    6.6 MB output (in repo)
├── PROJECT.md                         Architectural decisions log
├── SESSION.md                         Append-only build log + handoff notes
└── README.md                          Quickstart + roadmap
```

---

## What works today (steps 1-7 complete)

| # | Step | Status | Notes |
|---|------|--------|-------|
| 1 | ROS2 workspace scaffold | ✓ | 8 packages, clean colcon build |
| 2 | Gazebo room + anchors + messages | ✓ | 5 anchors latched, xacro toggles for ceiling/furniture |
| 3a-i | Sionna scene + single-point test | ✓ | 0.24cm range error vs Euclidean, LOS detection works |
| 3a-ii | Sionna grid sweep (386k points × 5 anchors) | ✓ | 6.6 MB HDF5, sub-mm LOS accuracy |
| 3b | ROS runtime range publisher | ✓ | 10 Hz, DW3000-realistic noise, LOS flag, variance |
| 4 | Trilateration node | ✓ | Variance-weighted LM LSQ, 5-6cm median LOS error |
| 5 | Kalman filter | ✓ | 6D constant-velocity KF, publishes `/microuwb/position_filtered` |
| 6 | Drone URDF + Gazebo plugins | ✓ | Sub-50g quad URDF, `libgazebo_ros_force.so` motor plugins |
| 7 | PID flight controller | ✓ | Cascaded outer-position + inner-attitude PID, rqt_reconfigure setpoints |

**Key validated metrics:**
- Sionna LOS range error: median <0.3 cm, p95 <0.4 cm
- Trilateration LOS: median 5-6 cm, p95 8-22 cm
- Without variance weighting: median spikes to 94 cm (confirms weighting essential)
- Publish rate: stable 10.000 Hz with ±0.3ms jitter

---

## What's next (steps 8-9)

| # | Step | Est. effort | Notes |
|---|------|-------------|-------|
| 8 | Waypoint nav + wall avoidance | small | Mostly state machine |
| 9 | Full demo launch + recording | small | Glue + rosbag recording |

---

## Quickstart for teammates

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop` + `ros-humble-gazebo-ros-pkgs`)
- Gazebo Classic 11
- Anaconda with a `sionna_env` conda env (Sionna 1.2.1) — only needed if
  regenerating the HDF5 table

### Clone and build

```bash
git clone https://github.com/<user>/microuwb-nav.git
cd microuwb-nav
colcon build --symlink-install
source install/setup.bash
```

### Run what exists today

**Just the Gazebo room + anchors:**
```bash
ros2 launch microuwb_bringup world.launch.py
# or with ceiling disabled for easier camera view:
ros2 launch microuwb_bringup world.launch.py ceiling:=false
```

**Full UWB pipeline with test trajectory (steps 1-4):**
```bash
ros2 launch microuwb_bringup uwb_sim.launch.py test_trajectory:=true
```

This starts:
- Gazebo with the room
- `anchor_publisher` (latches 5 anchor positions)
- `uwb_range_simulator` (interpolates HDF5 → ranges)
- `trilateration_node` (ranges → position estimate)
- `test_pose_publisher` (figure-8 around room center)
- `verify_trilateration` (logs median/p95 error vs ground truth)

**Verify the pipeline:**
```bash
ros2 topic hz /microuwb/ranges            # should be 10.000 Hz
ros2 topic hz /microuwb/position_estimate  # should be 10.000 Hz
ros2 topic echo /microuwb/position_estimate
```

`verify_trilateration` prints median/p95 error every 2 seconds. Expect
median under 10 cm in default conditions.

**Full sim with drone + PID flight controller (steps 1-7):**
```bash
ros2 launch microuwb_bringup uwb_sim.launch.py ceiling:=false use_drone:=true
```

This additionally starts:
- `kalman_filter` (always on — publishes `/microuwb/position_filtered`)
- `flight_controller` (cascaded PID — arms after 5 s, one-shot)
- `gt_relay` (republishes Gazebo ground truth as PoseStamped)
- `static_transform_publisher` (world → map TF so RViz works)

**If the drone crashes / tips over — respawn:**
```bash
bash ~/microuwb_nav_ws/reset_drone.sh
```

**Tune setpoints and PID gains live:**
```bash
ros2 run rqt_reconfigure rqt_reconfigure
# Under flight_controller:
#   setpoint_z → slide to 0.8 m first (activates XY loop above 0.3 m)
#   setpoint_x / setpoint_y → move drone horizontally
#   hover_thrust → adjust if drone sinks or rises at idle
#   kp_pos_z / kd_pos_z / ki_pos_z → Z response tuning
#   kp_pos_xy / kd_pos_xy / ki_pos_xy → XY response tuning
#   max_tilt_rad (default 0.15) / max_accel (default 0.5) → aggressiveness
```

**Switch between ground-truth and KF positioning (default: ground truth):**
```bash
ros2 param set /flight_controller use_ground_truth false
```

**Visualise in RViz:**
```bash
rviz2
# Fixed Frame: map
# Add → By topic:
#   /drone/ground_truth_pose  (Pose)   — red arrow = drone position
#   /microuwb/setpoint_marker (Marker) — green sphere = current setpoint
#     pure green  = within 0.3 m (holding)
#     teal/cyan   = navigating toward target
```

**Debug controller internals (15 fields at 100 Hz):**
```bash
ros2 topic echo /microuwb/controller_debug
# Field order: sp_x sp_y sp_z  pos_x pos_y pos_z
#              roll pitch yaw  des_roll des_pitch
#              roll_t pitch_t yaw_t  thrust
```

**Compare Ground Truth vs Kalman Filter (TA demo):**
```bash
# Recommended — PlotJuggler (install once):
sudo nala install ros-humble-plotjuggler-ros

ros2 run plotjuggler plotjuggler
# In PlotJuggler: Streaming → ROS2 Topic Subscriber
# Drag /drone/ground_truth_pose and /microuwb/position_filtered onto the same plot
```

Fallback with rqt_plot:
```bash
ros2 run rqt_plot rqt_plot \
  /drone/ground_truth_pose/pose/position/x \
  /microuwb/position_filtered/pose/position/x \
  /drone/ground_truth_pose/pose/position/y \
  /microuwb/position_filtered/pose/position/y \
  /drone/ground_truth_pose/pose/position/z \
  /microuwb/position_filtered/pose/position/z
```

### Regenerate the HDF5 table (only if scene changes)

```bash
conda activate sionna_env
cd sionna_precompute
python scripts/precompute_cir_table.py      # ~30-60 min on RTX 4060
python scripts/validate_table.py             # produces diagnostic plots
```

---

## Key architectural decisions (locked)

1. **TWR architecture only.** No AoA, TDoA, or PDoA. Matches the target
   DW3000 hardware.
2. **Sionna → HDF5 → ROS.** Offline ray tracing, runtime interpolation.
   Decouples physics simulation from ROS runtime entirely. No conda/ROS
   environment coexistence problems.
3. **Core Electronics UWB tutorial as source of truth** for trilateration
   and Kalman math. We deviate only with explicit reasoning (e.g. scipy
   LSQ instead of Cramer's rule because we have numpy, not MicroPython).
4. **Interface contracts before implementation.** The ROS message types
   (UWBRange, UWBRangeArray) and topic names were locked before any of
   the sensor/algorithm nodes were built. Makes each step swappable.
5. **Staged verification.** Every step has explicit numeric acceptance
   criteria verified before the next step starts. No stacked unverified
   work.
6. **Orchestrator/executor development pattern.** Architecture decisions
   in Claude Opus chat; code written by Sonnet via Claude Code with
   scoped prompts. PROJECT.md and SESSION.md are the persistent state
   across sessions.

---

## Known issues / backlog

- `rx_power` field in UWBRange is NaN — `first_path_power` extraction
  from Sionna returns NaN for some cells; not used in trilateration math.
- TWR sequential anchor staggering not modeled (all 5 fire with same
  timestamp). Kalman filter handles residual timing effects.
- NLOS extra runtime noise implemented but disabled by default. Toggle
  `enable_nlos_extra_noise: true` in `uwb_simulator.yaml` to activate.
- **Flight controller arm timer** fires every 5s (periodic, not one-shot) —
  setpoint chases drifted position on each re-arm. Acceptable for demo.
- **XY control gate** at `pos[2] > 0.3m` — slide `setpoint_z` above 0.3m
  first before using X/Y sliders, otherwise XY loop is suppressed.

---

## References

- [Core Electronics UWB 3D tutorial](https://core-electronics.com.au/guides/sensors/diy-2d-and-3d-spatial-tracking-with-ultra-wideband-arduino-and-pico-guide) — trilateration + Kalman math reference
- [Sionna RT 1.2.1 docs](https://nvlabs.github.io/sionna/api/rt.html)
- [Qorvo DW3000](https://www.qorvo.com/products/p/DWM3000) — target UWB hardware

---

## Questions? Contact

Project lead: Amogh Singh (check repo for latest contact).
For continuity context across Claude sessions, see PROJECT.md + SESSION.md.
