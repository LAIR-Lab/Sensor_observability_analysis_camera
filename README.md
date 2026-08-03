## Work in progress

# Sensor_observability_analysis_camera
An open-source ROS 2 implementation of Sensor Observability Analysis and demo for USB cameras viewpoints through observability metrics and Jacobian-based motion planning.

## Features

- SOA metric computation
- Camera Jacobian calculation
- TF-based camera tracking
- Gradient ascent optimization
- Gazebo simulation

## Requirements

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- Python 3.12

## Build
Build the latest version via:
```bash

cd ~/<ros2_ws>
git clone 
colcon build
source install/setup.bash
```
## Usage
The usage of this package on a demo so-101 robot requires 1 launch file and 2 ros2 nodes in required order.

1. Run the `gazebo.launch.py` launch file in `digital_twin` to generate the so-101 robot in rviz2 and gazebo. Initialize ros2-gazebo plugins,controllers.
```bash
ros2 launch digital_twin gazebo.launch.py
```

2. Run the `soa_camera_jacobian_node` launch file next to calculate the SOA-jacobian for each joints.
```bash
ros2 run sensor_observability_analysis_py soa_camera_jacobian_node
```

3. Finally run `` launch file to use the controller which utilizes the SOA-jacobian values to move the robot to obtain higher
```bash
ros2 run sensor_observability_analysis_py soa_cam_move_node
```

# SOA — Sensor Observability Analysis

Two ROS 2 nodes that move a camera-equipped robot arm to get a better view of
a point of interest (POI), using gradient ascent on a visibility score.

- **`soa_jacobian_cam_node.py`** — computes the score and its gradient.
- **`soa_cam_move.py`** — moves the arm to climb that gradient.

They only talk to each other over topics, so each can run/restart independently.

---

## How it works

1. The **Jacobian node** looks at the camera pose and the POI position (via
   TF) and computes:
   - `S` — a score from ~1 (POI dead-center in view) down through 0 and
     negative as the POI leaves the field of view.
   - `J_SOA` — the gradient of `S` with respect to each joint angle.
2. The **motion node** reads `S` and `J_SOA` and nudges each joint in the
   direction that increases `S`, sending trajectory goals until the score is
   "good enough" or the highest number of steps is reached.

```
/joint_states ──► Jacobian node ──► /soa/value, /soa/jacobian ──► Motion node ──► arm trajectory
                       ▲                                                │
                  TF: camera + POI pose                          adjusts joint angles
```

---

## `soa_camera_jacobian_node`

**Node:** `soa_camera_jacobian` · **Rate:** 100 Hz (10 ms timer)

**Subscribes:** `/joint_states`
**Publishes:** `/soa/value` (S), `/soa/jacobian` (J_SOA), `/soa/theta` (raw angle)
**Uses TF:** `base_link → camera_3` (camera pose), `world → red_sphere` (POI)

**Score formula:**
```
θ = angle between camera axis and direction to POI
S = 1 - θ / fov          (fov = 60°, not clamped — can go negative)
```
`J_SOA` is the chain rule `dS/dxc @ Jc`, where `Jc` is the robot's Jacobian to
`camera_3` and `dS/dxc` is the hand-derived gradient of `S` w.r.t. camera's 
position/orientation.

**Key Points:**
- Camera's optical axis is the local **+X** direction of `camera_3`.
- `S` is deliberately **not clamped** to 0 outside the FOV — keeps the
  gradient useful even when the POI is out of view.
---

## `soa_cam_move_node`

**Node:** `soa_gradient_ascent` · **Rate:** 50 Hz (20 ms timer)

**Subscribes:** `/joint_states`, `/soa/value`, `/soa/jacobian`
**Sends actions to:** `/arm_controller/follow_joint_trajectory`

**Active joints:** `joint_0`–`joint_4` (the 5 joints on the camera's
kinematic chain). `joint_5` isn't part of the chain () and is just held in
place.

**Each cycle:**
1. Stop if `S >= 0.95` (good enough) or 1000 iterations have passed.
2. Skip any joint that's at its limit (or previously found physically
   blocked) in the direction it wants to move.
3. Move **all** usable joints at once, each scaled by its own gradient
   strength relative to the strongest one.
4. If `S` drops after a move → shrink step size and flip that joint's
   direction (in case its sign was wrong). If `S` climbs → grow the step
   back up.
5. Near the goal (`S >= 0.85`) → slow down to avoid overshooting.
6. If stuck (flat for 8 ticks at minimum step) → try a random "kick" to
   escape the plateau, up to 50 times, then fall back to the best pose seen.

**Key Points:**
- To disable a feature (basin hopping), its parameter is set to 0 (e.g. `max_kicks = 0`)
  rather than commenting out code — that's the convention used throughout.
- A `/soa/jacobian` message with the wrong length (should be 5 for camera_3, one per
  active joint) is silently dropped — no warning logged. If gradients never
  seem to arrive, check the Jacobian node is publishing the right shape.
- All the actual decision-making (direction, step size, stall/kick recovery)
  lives in this file. The Jacobian node is solely responsible for jacobians.

---

## Key parameters (motion node)

| Parameter | Default | What it does |
|---|---|---|
| `step` / `base_step` | 1° | Angular step per move, regrows to this after shrinking |
| `min_step` | 0.1° | Floor for step size |
| `soa_acceptable` | 0.95 | Score at which the search stops |
| `soa_slowdown_threshold` | 0.85 | Score at which step/speed slow down |
| `max_iterations` | 1000 | Hard stop on the whole run |
| `max_kicks` | 50 | Random perturbations tried when stuck |
| `kick_deg` | 5° (2°–20° range) | Size of each random kick |
| `stall_patience` | 8 | Flat ticks before declaring a stall |
