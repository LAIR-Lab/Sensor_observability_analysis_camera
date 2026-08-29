# Sensor Observability Analysis — Camera (SOA)

> **Status: Stable**

An open-source ROS 2 implementation of **Sensor Observability Analysis (SOA)** for
USB-camera viewpoints. Two nodes work together to move a camera-equipped robot arm
toward a better view of a point of interest (POI), using gradient ascent on a
visibility score — demonstrated on a SO-101 arm in Gazebo.

## Features

- SOA visibility score and Jacobian computation
- TF-based camera and POI tracking
- Gradient ascent viewpoint optimization
- Gazebo Harmonic simulation with RViz2 visualization

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12

## Build

```bash
cd ~/<ros2_ws>
git clone https://github.com/LAIR-Lab/Sensor_observability_analysis_camera.git
colcon build
source install/setup.bash
```

## How it works

The Jacobian node computes a visibility score and its gradient; the motion node
climbs that gradient by repeatedly nudging joints and sending trajectory goals.
The two nodes only communicate over topics, so each can be run or restarted
independently.

```
/joint_states ──► Jacobian node ──► /soa/value, /soa/jacobian ──► Motion node ──► arm trajectory
                       ▲                                                │
                  TF: camera + POI pose                          adjusts joint angles
```

1. **Jacobian node** looks up the camera pose and POI position via TF and computes:
   - `S` — a score from ~1 (POI dead-center in view) down through 0 and negative
     as the POI leaves the field of view.
   - `J_SOA` — the gradient of `S` with respect to each joint angle.
2. **Motion node** reads `S` and `J_SOA` and nudges each joint in the direction
   that increases `S`, sending trajectory goals until the score is "good enough"
   or the iteration limit is reached.

## Usage
The simulation is divided into 2:
One is with a static point of interest(POI) another is with a moving point of interest(POI)

### For a static POI:

Package: `sensor_observability_analysis_py`. Run in this order:

**1. Launch the simulation** — spawns the SO-101 robot in Gazebo and RViz2, and
initializes `ros2_control` plugins/controllers.

```bash
ros2 launch digital_twin gazebo.launch.py
```

**2. Start the Jacobian node** — computes SOA and its gradient for each joint.

```bash
ros2 run sensor_observability_analysis_py soa_camera_jacobian_node
```

**3. Start the motion node** — uses the SOA Jacobian to move the robot toward
higher visibility.

```bash
ros2 run sensor_observability_analysis_py soa_cam_move_node
```

### For a moving POI

Package: `sensor_observability_analysis_py`. Run in this order:

**1. Launch the simulation** — spawns the SO-101 robot in Gazebo and RViz2, and
initializes `ros2_control` plugins/controllers.

```bash
ros2 launch digital_twin gazebo2.launch.py
```

**2. Start the Jacobian node** — computes SOA and its gradient for each joint.

```bash
ros2 run sensor_observability_analysis_py soa_camera_obs_jacobian_node
```

**3. Start the motion node** — uses the SOA Jacobian to move the robot toward
higher visibility.

```bash
ros2 run sensor_observability_analysis_py soa_cam_obs_move_node
```
## Nodes

### `soa_camera_jacobian_node`

| | |
|---|---|
| **Node name** | `soa_camera_jacobian` |
| **Rate** | 100 Hz (10 ms timer) |
| **Subscribes** | `/joint_states` |
| **Publishes** | `/soa/value` (S), `/soa/jacobian` (J_SOA), `/soa/theta` (raw angle) |
| **Uses TF** | `base_link → camera_3` (camera pose), `world → red_sphere` (POI) |

**Score formula:**

```
θ = angle between camera axis and direction to POI
S = 1 - θ / fov          (fov = 60°, not clamped — can go negative)
```

Though the theoretical formula for it is :


$$
T_{\text{uni}}(\hat{\mathbf{s}},\mathbf{p}_{\mathrm{poi}})=
\left(
\frac{\theta^{i}_{\mathrm{FOV}}-\theta^{i}}
{\theta^{i}_{\mathrm{FOV}}}
\right)
$$

$$
s^{i}=
\left(
\frac{\overline{\mathrm{FOV}}^{i}-\theta^{i}}
{\overline{\mathrm{FOV}}^{i}}
\right)
$$

For Obstacles Occlusions:

$$
\tilde{\mathbf{s}}^{i}=
\left(
T_{\text{uni}}(\hat{\mathbf{s}},\mathbf{p}_{\mathrm{poi}})
\-
\textstyle\sum_i \lambda_i
T_{\text{uni}}(\mathbf{p}_{\mathrm{poi}},\mathbf{p}_{\mathrm{obs}})
\right)
$$

The practical formual removes the max function to deal with extreme cases when the angle is more than 60 /degrees

`J_SOA` is the chain rule `dS/dxc @ Jc`, where `Jc` is the robot's Jacobian to
`camera_3` and `dS/dxc` is the hand-derived gradient of `S` with respect to the
camera's position/orientation.

**Key points:**
- The camera's optical axis is the local **+X** direction of `camera_3`.
- `S` is deliberately **not clamped** to 0 outside the FOV — this keeps the
  gradient useful even when the POI is out of view.

### `soa_cam_move_node`

| | |
|---|---|
| **Node name** | `soa_gradient_ascent` |
| **Rate** | 50 Hz (20 ms timer) |
| **Subscribes** | `/joint_states`, `/soa/value`, `/soa/jacobian` |
| **Sends actions to** | `/arm_controller/follow_joint_trajectory` |
| **Active joints** | `joint_0`–`joint_4` (the 5 joints on the camera's kinematic chain) |

`joint_5` is not part of the camera's kinematic chain and is simply held in place.

**Each control cycle:**
1. Stop if `S >= soa_acceptable` (good enough) or the iteration limit is reached.
2. Skip any joint that's at its limit — or previously found physically
   blocked — in the direction it wants to move.
3. Move all usable joints at once, each scaled by its gradient strength
   relative to the strongest one.
4. If `S` drops after a move → shrink step size and flip that joint's
   direction (in case its sign was wrong). If `S` climbs → grow the step
   back up.
5. Near the goal (`S >= soa_slowdown_threshold`) → slow down to avoid overshooting.
6. If stuck (flat for `stall_patience` ticks at minimum step) → try a random
   "kick" to escape the plateau, up to `max_kicks` times, then fall back to
   the best pose seen.

**Key points:**
- To disable a feature (e.g. basin hopping), set its parameter to `0`
  (e.g. `max_kicks = 0`) rather than commenting out code — this is the
  convention used throughout.
- A `/soa/jacobian` message with the wrong length (should be 5, one per
  active joint) is silently dropped with no warning logged. If gradients
  never seem to arrive, check that the Jacobian node is publishing the
  right shape.
- All decision-making (direction, step size, stall/kick recovery) lives in
  this node. The Jacobian node is solely responsible for computing Jacobians.

## Key parameters (motion node)

| Parameter | Default | What it does |
|---|---|---|
| `step` / `base_step` | 1° | Angular step per move; regrows to this after shrinking |
| `min_step` | 0.1° | Floor for step size |
| `soa_acceptable` | 0.95 | Score at which the search stops |
| `soa_slowdown_threshold` | 0.85 | Score at which step/speed slow down |
| `max_iterations` | 1000 | Hard stop on the whole run |
| `max_kicks` | 50 | Random perturbations tried when stuck |
| `kick_deg` | 5° (2°–20° range) | Size of each random kick |
| `stall_patience` | 8 | Flat ticks before declaring a stall |

# Handling multiple Point of Interests(POI)
Till now only single POI was being handles by a single uni-directional sensor. We extended the SOA's capability to include multiple cameras by modifying the equation as follows:

$$
S_{agg} = \Gamma_{\min}^{k}(\mathbf{S}) = -\frac{1}{k} \ln \left( \sum_{c=1}^{N} e^{-k S_c} \right)
$$

$$
w_c = \frac{e^{-k S_c}}{\displaystyle\sum_{c'=1}^{N} e^{-k S_{c'}}}, \qquad
J_{agg} = \sum_{c=1}^{N} w_c J_c
$$

$$\text{softmin}_k(\mathbf{S})$$ or $$\text{LSE}_{-k}(\mathbf{S})$$ is used in SOA to include the weightage for all available sensors's SOA values, something which hardmax failed to achieve.
The aggregate Jacobian calculates the best trajectory goals including multiple Jacobians from $$ **N** $$ different cameras mounted on the robot.

Basically the weakest

# Bonus Primitive SOA (for Bi-directional sensors)

This repo is also bundled with an implimentation of Force-Torque sensors which looksup for the observability metric from the "under arm" of the so=101 robot to the "upper_arm" link.

See the [SOA_FT README](docs/SOA_ft_sensor.md).
