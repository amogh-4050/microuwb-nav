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
| 3a-ii | CIR grid sweep (93×73×57, 5 anchors), HDF5 lookup table, validation plots | ✓ done |
| 3b | ROS range publisher — reads HDF5, interpolates at drone pose, adds noise | todo |
| 4 | Trilateration node — 5-anchor NLLS or weighted least squares | todo |
| 5 | Kalman filter node — fuses UWB position fix + IMU odometry | todo |
| 6 | Drone URDF + Gazebo plugin (propeller forces, IMU, ground truth) | todo |
| 7 | Flight controller — cascaded PID, attitude inner loop + position outer loop | todo |
| 8 | Waypoint navigation + wall avoidance | todo |
| 9 | Full system integration launch + closed-loop demo | todo |
| 10 | Physical hardware bringup (nRF52840 + DW3000 + custom PCB) | future |

**Validated so far:** step 3a-i single-point CIR — first-path range 3.4112 m vs Euclidean 3.4088 m, error 0.24 cm; LOS correctly detected. Step 3a-ii grid sweep and validation pipeline written; run `scripts/precompute_cir_table.py` to generate HDF5.

---

## Quickstart

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble (`ros-humble-desktop` + `ros-humble-gazebo-ros-pkgs`)
- Gazebo Classic 11
- Anaconda with a `sionna_env` conda environment (Sionna 1.2.1, see `sionna_precompute/requirements.txt`)

### Simulation (Gazebo room + anchors)

```bash
cd ~/microuwb_nav_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch microuwb_bringup world.launch.py
```

Optional args:
```bash
ros2 launch microuwb_bringup world.launch.py ceiling:=false furniture:=false
```

Verify anchor topic (TransientLocal QoS required):
```bash
ros2 topic echo /microuwb/anchors \
  --qos-durability transient_local --qos-reliability reliable --once
```

### Sionna RT notebooks

```bash
source ~/anaconda3/bin/activate
conda activate sionna_env
cd ~/microuwb_nav_ws/sionna_precompute
jupyter notebook
```

Open `notebooks/01_scene_sanity.ipynb` first, then `02_single_point_test.ipynb`.
Rendered images are saved to `sionna_precompute/renders/`.

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
