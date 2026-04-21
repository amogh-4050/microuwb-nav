# MicroUWB-Nav

GPS-denied indoor navigation for a sub-50g quadcopter, driven by UWB ranging and Sionna RT channel simulation.

---

## What this is

A simulation-first proof-of-concept for a micro quadcopter that navigates indoors using Ultra-Wideband (UWB) time-of-flight ranging. Five DW3000 anchors are fixed to the room walls; the drone measures two-way ranges to each anchor, runs trilateration to get a 2D/3D position fix, and feeds that into a Kalman filter for smooth state estimates.

The simulation pipeline is built on **ROS2 Humble + Gazebo Classic** for the runtime and **Sionna RT 1.2.1** for offline channel impulse response (CIR) precomputation. Sionna traces multipath rays through a Mitsuba 3 scene of the room, producing an HDF5 lookup table of first-path delays at every grid point. The ROS runtime interpolates this table to simulate realistic UWB range measurements — including multipath-induced bias — without running a full ray tracer in the control loop.

Hardware target: nRF52840 + DW3000 modules on a custom PCB, LSM6DS3 IMU for attitude, all-up weight under 50g. The simulation is designed so that the trilateration and Kalman nodes can be swapped onto the physical drone with minimal changes.

---

## Architecture

```
  ┌─────────────────────────────────────────────────────────────┐
  │  Offline (conda / sionna_env)                               │
  │                                                             │
  │  Sionna RT 1.2.1  ──►  CIR HDF5 table                      │
  │  (Mitsuba 3 scene)      (anchor × grid point × delay)      │
  └───────────────────────────────┬─────────────────────────────┘
                                  │ data/cir_lookup.h5
  ┌───────────────────────────────▼─────────────────────────────┐
  │  ROS2 Runtime (Humble)                                       │
  │                                                             │
  │  HDF5 interpolator ──► /microuwb/ranges  (UWBRangeArray)    │
  │                              │                              │
  │                      trilateration node                     │
  │                              │                              │
  │                       Kalman filter node                    │
  │                              │                              │
  │                    /microuwb/pose_estimate                  │
  │                              │                              │
  │                    flight controller (PID)                  │
  │                              │                              │
  │                       Gazebo drone model                    │
  └─────────────────────────────────────────────────────────────┘
```

---

## Repo layout

```
microuwb_nav_ws/
├── src/
│   ├── microuwb_msgs/         Custom ROS2 messages (UWBRange, UWBRangeArray, Anchor, AnchorArray)
│   ├── microuwb_gazebo/       5×4×3m Gazebo Classic room, 5 UWB anchor models, launch files
│   ├── microuwb_uwb/          Range simulator (HDF5 interpolator), trilateration, EKF
│   ├── microuwb_control/      Attitude + position PID controllers
│   ├── microuwb_navigation/   Waypoint planner, wall avoidance, return-to-home
│   ├── microuwb_description/  Drone URDF / SDF model
│   ├── microuwb_bringup/      Top-level launch files and config (anchors.yaml)
│   └── microuwb_rviz/         RViz2 display configs
│
├── sionna_precompute/         Offline CIR pipeline (Sionna 1.2.1, conda env)
│   ├── scene/                 Mitsuba 3 scene XML + PLY meshes + mesh generator
│   ├── notebooks/             Sanity-check notebooks (01, 02)
│   ├── scripts/               (step 3a-ii) Headless grid sweep
│   └── data/                  (step 3a-ii) HDF5 output
│
├── PROJECT.md                 Architecture notes
└── SESSION.md                 Build log and step-by-step progress
```

Future: `firmware/` (nRF52840 Zephyr code), `hardware/` (KiCad PCB).

---

## Status / roadmap

| Step | Description | Status |
|------|-------------|--------|
| 1 | ROS2 workspace scaffold — 8 packages, colcon build clean | ✓ done |
| 2 | Gazebo room + 5 UWB anchors + custom messages + latched anchor publisher | ✓ done |
| 3a-i | Sionna RT scene (Mitsuba 3 room, ITU concrete + wood furniture) + single-point CIR sanity test | ✓ done |
| 3a-ii | CIR grid sweep (93×73×57, 5 anchors), HDF5 lookup table (6.6 MB) | ✓ done |
| 3b | ROS range publisher — HDF5 trilinear interpolation, DW3000 noise model, 10 Hz | ✓ done |
| 4 | Trilateration node — weighted nonlinear LSQ (LM), median ~5 cm, p95 ~8 cm | ✓ done |
| 5 | EKF node — fuses UWB position fix + IMU odometry | todo |
| 6 | Drone URDF + Gazebo plugin (propeller forces, IMU, ground truth) | todo |
| 7 | Flight controller — cascaded PID, attitude inner loop + position outer loop | todo |
| 8 | Waypoint navigation + wall avoidance | todo |
| 9 | Full system integration launch + closed-loop demo | todo |
| 10 | Physical hardware bringup (nRF52840 + DW3000 + custom PCB) | future |

**Validated so far:**
- Step 3a-i: first-path range error 0.24 cm vs Euclidean; LOS correctly detected.
- Step 3a-ii: 93×73×57 grid, 5 cm resolution, LOS% > 85%, p95 range error < 10 cm.
- Step 3b: `/microuwb/ranges` at 10.000 Hz, per-anchor bias + 2 cm clock noise, variance 0.005 (LOS) / 0.050 (NLOS) m².
- Step 4: trilateration at 10 Hz; LOS median ~5 cm, p95 ~8 cm; NLOS zones median ≤ 12 cm, p95 ≤ 22 cm. A/B confirmed variance weighting essential (without it p95 spikes to 107–118 cm in NLOS).

---

## Quickstart

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop` + `ros-humble-gazebo-ros-pkgs`)
- Gazebo Classic 11
- Anaconda with a `sionna_env` conda environment (Sionna 1.2.1, see `sionna_precompute/requirements.txt`)

### Build

```bash
cd ~/microuwb_nav_ws
colcon build --symlink-install
source install/setup.bash
```

### Gazebo room + anchors only

```bash
ros2 launch microuwb_bringup world.launch.py
# optional: ros2 launch microuwb_bringup world.launch.py ceiling:=false furniture:=false
```

Verify anchor topic (TransientLocal QoS required):
```bash
ros2 topic echo /microuwb/anchors \
  --qos-durability transient_local --qos-reliability reliable --once
```

### UWB range simulator + trilateration (steps 3b + 4)

Full stack with figure-8 test trajectory and live error stats:
```bash
ros2 launch microuwb_bringup uwb_sim.launch.py test_trajectory:=true
```

Check outputs:
```bash
ros2 topic hz /microuwb/ranges              # should be 10.000 Hz
ros2 topic hz /microuwb/position_estimate   # should be 10.000 Hz
ros2 topic echo /microuwb/position_estimate # PoseStamped x/y/z estimates
```

The `verify_trilateration` node (started automatically with `test_trajectory:=true`) logs
median/p95 error vs ground truth every 2 seconds. Expected: median < 30 cm, p95 < 1 m.

Individual nodes (if launching manually):
```bash
ros2 run microuwb_uwb anchor_publisher
ros2 run microuwb_uwb uwb_range_simulator \
  --ros-args -p cir_table_path:=/path/to/room_cir_table_full.h5
ros2 run microuwb_uwb trilateration_node
ros2 run microuwb_uwb test_pose_publisher   # figure-8 test trajectory
ros2 run microuwb_uwb verify_trilateration  # error stats
```

### Sionna RT — regenerate HDF5 table (offline, GPU required)

```bash
conda activate sionna_env
cd ~/microuwb_nav_ws/sionna_precompute
python scripts/precompute_cir_table.py      # ~hours on RTX 4060, writes data/room_cir_table_full.h5
```

Sanity notebooks:
```bash
jupyter notebook
# 01_scene_sanity.ipynb → 02_single_point_test.ipynb → 03_path_visualization.ipynb
```

---

## Tech stack

- **ROS2 Humble** — middleware, message passing, launch system
- **Gazebo Classic 11** — physics simulation, sensor plugins
- **Sionna RT 1.2.1** — GPU-accelerated ray tracing for UWB channel simulation
- **Mitsuba 3 / Dr.JIT** — Sionna's rendering backend
- **Python 3.10** — all ROS nodes and Sionna scripts
- **HDF5 (h5py)** — offline CIR lookup table storage
- Future firmware: **nRF52840** (Zephyr RTOS), **DW3000** (UWB transceiver), **LSM6DS3** (IMU)

---

## References

- [Qorvo DW3000 product page](https://www.qorvo.com/products/p/DWM3000) — UWB transceiver used in the target hardware design
- [Sionna RT documentation](https://nvlabs.github.io/sionna/api/rt.html) — Ray tracing API reference for channel simulation
- [Core Electronics UWB indoor positioning tutorial](https://core-electronics.com.au/guides/uwb-indoor-positioning-with-esp32/) — TWR protocol and trilateration approach adapted for this project

---

## License

MIT — see [LICENSE](LICENSE).
