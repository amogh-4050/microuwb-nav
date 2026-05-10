## Handoff note (2026-04-21, end of orchestrator session)

**Project status: Step 4 complete.** Trilateration working with
variance-weighted LM nonlinear LSQ. Median error 5-6cm in LOS, p95 8-22cm.
A/B verified variance weighting matters (94cm median without it). Graceful
degradation under NLOS extra noise. /microuwb/position_estimate at 10Hz.

**Next active step: 5 — Kalman filter.**

Architecture: subscribe to /microuwb/position_estimate, run 6D
constant-velocity KF (state = [x,y,z,vx,vy,vz], measurement = [x,y,z]),
publish smoothed estimate on /microuwb/position_filtered. Follow Core
Electronics tutorial's SimpleKalman3D structure verbatim for first pass
(including the ad-hoc velocity update — we decided to evaluate tutorial
math first, only upgrade if measurably worse).

Tutorial default params: Q=0.12, R=1.1, dt=0.10. These may need tuning
for our 5-6cm input noise vs tutorial's ~10cm real-hardware noise.

Target improvement: KF should cut median error by 2-3x vs trilateration
alone (target ~2-3cm median in steady-state LOS), and significantly
reduce the NLOS spike behavior visible with weighting disabled.

**Steps remaining after 5:**
- 6. Drone URDF + Gazebo plugins (biggest time risk; consider
  "cheat mode" position-commanded drone as fallback)
- 7. PID flight controller (attitude + position hold)
- 8. Waypoint nav + wall avoidance
- 9. Demo launch + recording

**Backlog unchanged from previous handoff:**
- extract_metrics first_path_power NaN — regenerate HDF5 later
- TWR anchor staggering not modeled
- NLOS extra noise implemented, disabled by default

**Key file locations:**
- sionna_precompute/data/room_cir_table_full.h5 (6.6MB)
- microuwb_uwb/microuwb_uwb/uwb_range_simulator.py (step 3b)
- microuwb_uwb/microuwb_uwb/trilateration.py (step 4)
- microuwb_uwb/microuwb_uwb/verify_trilateration.py (step 4)
- microuwb_bringup/config/uwb_simulator.yaml
