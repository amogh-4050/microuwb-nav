# MicroUWB-Nav

GPS-denied indoor navigation for a sub-50g quadcopter, driven by UWB ranging and Sionna RT channel simulation. Full closed-loop pipeline from offline ray tracing through trilateration, Kalman filtering, cascaded PID control, and autonomous waypoint navigation — all running in ROS2 + Gazebo.

---

## What it is

A simulation-first proof-of-concept for a micro quadcopter that navigates indoors using Ultra-Wideband (UWB) two-way ranging (TWR) to 5 fixed anchors. The drone measures distances to each anchor, trilaterates a 3D position fix, smooths it with a Kalman filter, and feeds that into a cascaded PID flight controller.

**Key design choice:** UWB channel impulse responses are precomputed offline with Sionna RT ray tracing and stored in an HDF5 lookup table. The ROS runtime interpolates this table to simulate realistic multipath behaviour — including NLOS bias — without running a ray tracer in the control loop. NLOS measurements are included in trilateration via Huber robust loss (down-weighted by residual magnitude) rather than dropped.

Hardware target: nRF52840 + DW3000 UWB transceiver, LSM6DS3 IMU, all-up weight under 50g. The trilateration and Kalman nodes are designed to run on the physical drone with minimal changes.

---

## Architecture

```
 Offline (conda / sionna_env, one-shot):
 ┌──────────────────────────────────────────────────────────┐
 │ Sionna RT 1.2.1 (Mitsuba 3 backend)                      │
 │   ray-trace 386k grid points × 5 anchors                 │
 │   → HDF5 CIR lookup table (6.6 MB)                       │
 │     fields: range_m, los, first_path_power_db,           │
 │             total_rx_power_db, variance_m2               │
 └──────────────────────────────┬───────────────────────────┘
                                │ data/room_cir_table_full.h5
 Runtime (ROS2 Humble):
 ┌──────────────────────────────▼───────────────────────────┐
 │ uwb_range_simulator                                       │
 │   trilinear interpolation into HDF5                       │
 │   DW3000-realistic noise + NLOS extra noise               │
 │   → /microuwb/ranges (UWBRangeArray @ 10 Hz)             │
 │                    │                                      │
 │ trilateration_node                                        │
 │   variance-weighted nonlinear LSQ (SciPy TRF + Huber)    │
 │   → /microuwb/position_estimate (PoseStamped @ 10 Hz)    │
 │                    │                                      │
 │ kalman_filter                                             │
 │   6D constant-velocity linear KF, standard update        │
 │   → /microuwb/position_filtered (PoseStamped @ 10 Hz)    │
 │                    │                                      │
 │ flight_controller                                         │
 │   cascaded PID: outer position (10 Hz) →                 │
 │   desired roll/pitch/thrust; inner attitude (100 Hz) →   │
 │   body torques; lookahead XY braking                      │
 │                    │                                      │
 │ Gazebo drone model (SDF + force/torque plugins)           │
 └──────────────────────────────────────────────────────────┘
```

Interface contracts (stable):
| Topic | Type | Rate | Publisher |
|-------|------|------|-----------|
| `/microuwb/anchors` | AnchorArray | latched | anchor_publisher |
| `/microuwb/ranges` | UWBRangeArray | 10 Hz | uwb_range_simulator |
| `/microuwb/position_estimate` | PoseStamped | 10 Hz | trilateration_node |
| `/microuwb/position_filtered` | PoseStamped | 10 Hz | kalman_filter |
| `/microuwb/setpoint` | PoseStamped | 5 Hz | waypoint_nav / rqt |
| `/drone/ground_truth_pose` | PoseStamped | 50 Hz | gt_relay |

---

## Room + anchor geometry

- Room: 5 m × 4 m × 3 m, concrete walls/floor/ceiling (ITU material model)
- Furniture: bookshelf, café table, side table as NLOS obstacles (`itu_wood`)
- Grid: 93 × 73 × 57 points at 5 cm resolution (386 541 points × 5 anchors)

| ID | Position (x, y, z) | Role |
|----|---------------------|------|
| a0 | (0.1, 0.1, 2.7) | ceiling corner |
| a1 | (4.9, 0.1, 2.7) | ceiling corner |
| a2 | (2.5, 3.9, 2.7) | ceiling mid-long-wall |
| a3 | (0.1, 2.0, 0.3) | floor mid-short-wall |
| a4 | (4.9, 2.0, 0.3) | floor mid-short-wall |

Source of truth: `src/microuwb_bringup/config/anchors.yaml`.

---

## Repo layout

```
microuwb_nav_ws/
├── src/
│   ├── microuwb_msgs/          UWBRange, UWBRangeArray, Anchor, AnchorArray
│   ├── microuwb_gazebo/        Room world (xacro), anchor models, world launch
│   ├── microuwb_uwb/           Range simulator, trilateration, Kalman filter,
│   │                           gt_relay, verify_trilateration, test_pose_publisher
│   ├── microuwb_control/       flight_controller, waypoint_nav,
│   │                           respawn_guard, test_setpoint_publisher
│   ├── microuwb_description/   Drone URDF/SDF model + mesh models
│   ├── microuwb_bringup/       Top-level launch files + YAML configs
│   │   └── config/             flight_controller.yaml, kalman_filter.yaml,
│   │                           trilateration.yaml, waypoint_nav.yaml, anchors.yaml
│   └── microuwb_rviz/          Pre-built RViz2 config (uwb_sim.rviz)
│
├── sionna_precompute/          Offline CIR pipeline (conda env, GPU required)
│   ├── scene/                  Mitsuba 3 room XML + PLY meshes
│   ├── notebooks/              Jupyter sanity + visualisation notebooks
│   ├── scripts/
│   │   ├── precompute_cir_table.py   Grid sweep → HDF5 (~30-60 min RTX 4060)
│   │   ├── validate_table.py         HDF5 validation + diagnostic plots
│   │   └── verify.py                 Quick single-point sanity check
│   └── data/room_cir_table_full.h5   6.6 MB HDF5 output (committed)
│
├── reset_drone.sh              Respawn drone to origin in Gazebo
└── README.md
```

---

## Status

| Step | Description | Status |
|------|-------------|--------|
| 1 | ROS2 workspace scaffold — 8 packages, clean colcon build | ✓ |
| 2 | Gazebo room + 5 UWB anchors + custom messages | ✓ |
| 3a-i | Sionna RT scene + single-point CIR sanity test | ✓ |
| 3a-ii | CIR grid sweep (386k pts × 5 anchors) → HDF5 | ✓ |
| 3b | ROS range publisher — HDF5 trilinear interpolation, DW3000 noise | ✓ |
| 4 | Trilateration node — Huber robust LSQ, NLOS included | ✓ |
| 5 | Kalman filter — 6D constant-velocity, standard update | ✓ |
| 6 | Drone URDF + Gazebo SDF model + ground truth relay | ✓ |
| 7 | Cascaded PID flight controller — outer position + inner attitude | ✓ |
| 8 | Waypoint nav — cuboid + circle trajectories, arrival-based advance | ✓ |
| 9 | Hardware bringup (nRF52840 + DW3000 + custom PCB) | future |

**Validated metrics:**
- Sionna LOS range error: median < 0.3 cm, p95 < 0.4 cm
- Trilateration LOS: median 5–6 cm, p95 8–10 cm
- Trilateration ≥1 NLOS anchor: median 5.9 cm (Huber, same as LOS)
- Without variance weighting: median spikes to 94 cm (confirms weighting essential)
- KF output: smooth, < 1 cm lag vs raw estimate at 10 Hz
- PID: stable hover ± 3 cm Z, XY arrival within arrival_radius_m = 0.30 m

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop` + `ros-humble-gazebo-ros-pkgs`)
- Gazebo Classic 11
- Python 3.10
- Anaconda with `sionna_env` conda env — only needed to regenerate the HDF5 table

```bash
# Install ROS2 extras if needed
sudo nala install ros-humble-plotjuggler-ros ros-humble-rqt-reconfigure
```

---

## Build

```bash
git clone https://github.com/amogh-4050/microuwb-nav.git microuwb_nav_ws
cd microuwb_nav_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the simulation

### Gazebo room + anchors only

```bash
ros2 launch microuwb_bringup world.launch.py ceiling:=false
```

Verify anchors (TransientLocal QoS required):
```bash
ros2 topic echo /microuwb/anchors \
  --qos-durability transient_local --qos-reliability reliable --once
```

### UWB pipeline verification — no drone (steps 3b + 4 + 5)

```bash
ros2 launch microuwb_bringup uwb_sim.launch.py test_trajectory:=true
```

Starts: Gazebo room, anchor_publisher, uwb_range_simulator, trilateration_node,
kalman_filter, test_pose_publisher (figure-8), verify_trilateration (error stats every 2 s).

```bash
ros2 topic hz /microuwb/ranges             # → 10.000 Hz
ros2 topic hz /microuwb/position_estimate  # → 10.000 Hz
ros2 topic echo /microuwb/position_estimate
```

`verify_trilateration` prints rolling median/p95 and a LOS vs NLOS breakdown showing
Huber robust loss effectiveness. Shutdown prints a full error histogram.

### Full simulation — drone + PID + waypoint nav (steps 6-8)

```bash
ros2 launch microuwb_bringup uwb_sim.launch.py \
  ceiling:=false use_drone:=true waypoint_nav:=true rviz:=true
```

This starts everything above plus:
- `spawn_drone` — spawns SDF model in Gazebo
- `gt_relay` — Gazebo odometry → `/drone/ground_truth_pose` (PoseStamped)
- `kalman_filter` — always on
- `flight_controller` — arms after 5 s, holds position, one-shot arm timer
- `respawn_guard` — auto-respawns drone if it leaves room bounds
- `waypoint_nav` — cuboid → circle trajectory, arrival-based, loops forever
- `rviz2` — opens with pre-built config (Grid, Pose, Odometry, Marker)
- `static_transform_publisher` — world → map identity TF for RViz

RViz displays:
- **Red arrow** = `/drone/ground_truth_pose` (drone position)
- **Orange trail** = `/drone/odom` (odometry history, keep=80)
- **Green sphere** = `/microuwb/setpoint_marker` (current target waypoint)
  - Pure green = drone within 0.30 m (holding)
  - Teal = navigating toward target

### Drone-only, manual setpoints (no waypoint sequencer)

```bash
ros2 launch microuwb_bringup uwb_sim.launch.py ceiling:=false use_drone:=true
```

Then tune setpoints and gains live:
```bash
ros2 run rqt_reconfigure rqt_reconfigure
# Under flight_controller:
#   setpoint_z → slide above 0.3 m first (activates XY loop)
#   setpoint_x / setpoint_y → move drone horizontally
#   hover_thrust, kp/kd/ki gains, max_tilt_rad, max_accel, lookahead_s
```

Switch positioning source (default: ground truth):
```bash
ros2 param set /flight_controller use_ground_truth false  # use KF output
```

Reset drone to spawn after a crash:
```bash
bash reset_drone.sh
# or publish manually:
ros2 topic pub --once /drone/reset std_msgs/msg/Empty {}
```

### Debug topics

```bash
# Controller internals at 100 Hz (15 fields):
ros2 topic echo /microuwb/controller_debug
# Order: sp_x sp_y sp_z  pos_x pos_y pos_z
#        roll pitch yaw  des_roll des_pitch
#        roll_t pitch_t yaw_t  thrust

# Trilateration skip counter on shutdown shows NLOS Huber stats
# Waypoint sequencer logs shape transitions:
# [waypoint_nav] wp 05/17 reached → advancing to 06  ── shape: cuboid → circle
```

### Compare ground truth vs Kalman filter (PlotJuggler)

```bash
ros2 run plotjuggler plotjuggler
# Streaming → ROS2 Topic Subscriber → Start
# Drag onto same plot:
#   /drone/ground_truth_pose/pose/position/x   (blue — truth)
#   /microuwb/position_estimate/pose/position/x (orange — raw trilateration)
#   /microuwb/position_filtered/pose/position/x (green — KF output)
# Remaining offset between truth and filtered = UWB positioning error (~5-8 cm)
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

---

## Tuned PID parameters (flight_controller.yaml)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `hover_thrust` | 0.57 N | feedforward; adjust if drone sinks/rises at idle |
| `kp/ki/kd_pos_z` | 3.0 / 0.06 / 1.5 | Z is direct-force — well-damped |
| `kp/ki/kd_pos_xy` | 3.0 / 0.06 / 1.5 | same ratio as Z; Huber braking compensates attitude lag |
| `lookahead_s` | 0.36 | predictive braking — start decelerating 360 ms before arrival |
| `max_tilt_rad` | 0.15 rad (~8.6°) | limits max horizontal force |
| `max_accel` | 2.0 m/s² | outer loop saturation limit |
| `kp/ki/kd_att_roll/pitch` | 0.29 / 0.005 / 0.016-0.017 | inner 100 Hz loop |
| `kp/ki/kd_att_yaw` | 0.05 / 0.002 / 0.009 | |
| `arm_delay_s` | 5.0 | one-shot; use reset_drone.sh to re-arm |

**XY lookahead explained:** The attitude loop adds ~150 ms lag between commanded tilt and
actual braking force. `lookahead_s = 0.36` makes the outer loop compute error against the
predicted position `pos + vel × 0.36`, so braking begins before the drone arrives rather
than after. Algebraically equivalent to adding `kp × lookahead` extra damping, applied
before the `max_accel` clamp so it works even when the command is saturated.

---

## Kalman filter parameters (kalman_filter.yaml)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `process_noise_q` | 0.01 | trust in constant-velocity model — lower = smoother |
| `measurement_noise_r` | 0.02 | trust in UWB measurements — calibrated to 5 cm trilateration noise |

The velocity states are updated by the standard `x = x_pred + K @ innovation` path —
the tutorial ad-hoc velocity rule was removed because it injected measurement noise
directly into velocity, causing a sawtooth position output.

---

## Trilateration parameters (trilateration.yaml)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `f_scale_m` | 0.30 | Huber loss knee — residuals > 0.30 m are soft outliers |
| `min_valid_ranges` | 4 | minimum anchors needed to attempt a solve |
| `weight_by_variance` | true | 1/√variance weighting from HDF5 |

NLOS measurements are **included**, not dropped. The solver uses `scipy.optimize.least_squares`
with `method='trf'` and `loss='huber'`. NLOS anchors with first-path delays 0.5–2 m longer
than Euclidean produce large residuals and are automatically down-weighted. Result: NLOS
cycles achieve the same 5.9 cm median as all-LOS cycles (validated in verify_trilateration).

---

## Regenerate the HDF5 table (only if scene changes)

```bash
conda activate sionna_env
cd sionna_precompute
python scripts/precompute_cir_table.py   # ~30-60 min on RTX 4060
python scripts/validate_table.py          # diagnostic plots + hard-fail thresholds
python scripts/verify.py                  # quick single-point sanity check
```

Sanity notebooks:
```bash
jupyter notebook
# 01_scene_sanity → 02_single_point_test → 03_path_visualization
```

---

## Known issues / backlog

- `rx_power` field in UWBRange is NaN — `first_path_power` extraction from Sionna
  returns NaN for some grid cells; not used in trilateration math so no impact.
- TWR sequential anchor staggering not modelled (all 5 fire with same timestamp).
  Kalman filter handles residual timing effects at 10 Hz.
- NLOS extra runtime noise implemented but disabled by default.
  Toggle `enable_nlos_extra_noise: true` in `uwb_simulator.yaml` to activate.
- XY arm timer is one-shot — use `reset_drone.sh` or `/drone/reset` to respawn.

---

## Key architectural decisions

1. **TWR only.** No AoA, TDoA, or PDoA. Matches the target DW3000 hardware which
   natively supports TWR.
2. **Sionna → HDF5 → ROS.** Offline ray tracing + runtime interpolation completely
   decouples physics simulation from the ROS environment. No conda/ROS coexistence
   issues; the HDF5 file is the stable interface.
3. **Huber robust loss, not LOS filter.** NLOS measurements are down-weighted by
   residual magnitude rather than dropped. This preserves geometry when fewer than
   3 LOS anchors are visible and gives the same accuracy as LOS-only filtering.
4. **Standard KF update for velocity.** The Core Electronics tutorial ad-hoc velocity
   rule `v = (pred - meas)/dt × 0.1` was replaced with the standard
   `x = x_pred + K @ innovation` because the ad-hoc rule injected raw measurement
   noise into velocity, which propagated back to position as sawtooth oscillation.
5. **Lookahead XY braking.** The outer position loop predicts drone position
   `pos + vel × lookahead_s` before computing XY error, compensating for attitude
   loop lag (~150 ms). This is the correct fix for cascaded-loop overshoot — not
   increasing kd beyond a point where it saturates alongside kp.
6. **Interface contracts before implementation.** ROS message types and topic names
   were locked before any algorithm nodes were built. Makes each step independently
   swappable.

---

## Tech stack

| Layer | Technology |
|-------|------------|
| Middleware | ROS2 Humble |
| Physics sim | Gazebo Classic 11 |
| Channel sim | Sionna RT 1.2.1 (GPU) |
| Ray-trace backend | Mitsuba 3 / Dr.JIT |
| Language | Python 3.10 |
| Table storage | HDF5 (h5py) |
| Optimisation | SciPy `least_squares` (TRF + Huber) |
| Future firmware | nRF52840 (Zephyr RTOS), DW3000, LSM6DS3 IMU |

---

## References

- [Qorvo DW3000](https://www.qorvo.com/products/p/DWM3000) — target UWB transceiver
- [Sionna RT 1.2.1 docs](https://nvlabs.github.io/sionna/api/rt.html) — ray-tracing API
- [Core Electronics UWB tutorial](https://core-electronics.com.au/guides/sensors/diy-2d-and-3d-spatial-tracking-with-ultra-wideband-arduino-and-pico-guide) — TWR + trilateration + Kalman math reference

---

## License

MIT — see [LICENSE](LICENSE).
