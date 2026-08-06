# Sensor Observability Analysis — Bi-Directional Sensors (SOA)

> **Status: stable**

An open-source ROS 2 implementation of **Sensor Observability Analysis (SOA)** for
**bi-directional sensors**. The framework computes an observability score from the
relative pose between two coordinate frames by constructing an observability matrix
and evaluating a simple observability metric.

## Features

- Bi-directional sensor observability model
- TF-based sensor geometry extraction
- Observability matrix construction
- Real-time SOA computation and visualization

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- NumPy
- SciPy

## Build

```bash
cd ~/<ros2_ws>
git clone https://github.com/LAIR-Lab/Sensor_observability_analysis_bidirectional.git
colcon build
source install/setup.bash
```

## How it works

The package consists of three ROS 2 nodes. The geometry node retrieves the relative
pose between two frames from TF, the solver constructs the observability matrix and
computes the SOA score, and the visualizer displays the resulting value.

```text
TF
 │
 ▼
sensor_geometry_node ──► /soa/sensor_geometry ──► soa_solver ──► /soa/index ──► soa_visualizer
```

The observability matrix is constructed using

$$
\mathbf{a}_i=\mathbf{R}\mathbf{a}_i^{\text{local}}
$$

$$
\boldsymbol{\tau}_i=\mathbf{a}_i\times\mathbf{r}
$$

$$
\mathbf{s}_i=
\begin{bmatrix}
\boldsymbol{\tau}_i\\
\mathbf{a}_i
\end{bmatrix},
\qquad
\mathbf{S}=
[\mathbf{s}_1,\mathbf{s}_2,\ldots,\mathbf{s}_6].
$$

The SOA index is then computed as

$$
s_i=\sum_j |S_{ij}|,
\qquad
\boxed{
\mathcal{O}=\prod_i s_i
}
$$

where larger values correspond to greater sensor observability.

## Usage

Package: `sensor_observability_analysis_py`

### 1. Start the geometry node

```bash
ros2 run sensor_observability_analysis_py sensor_geometry_node
```

### 2. Start the SOA solver

```bash
ros2 run sensor_observability_analysis_py soa_solver
```

### 3. Start the visualizer

```bash
ros2 run sensor_observability_analysis_py soa_visualizer
```

## Nodes

### `sensor_geometry_node`

| | |
|---|---|
| **Node name** | `sensor_geometry_node` |
| **Publishes** | `/soa/sensor_geometry` |
| **Uses TF** | Relative transform between the configured sensor and reference frames |

Computes the sensor position


$$
\mathbf{r}
\=
[x,y,z]^T
$$

and orientation

$$
\mathbf{R}.
$$

---

### `soa_solver`

| | |
|---|---|
| **Node name** | `soa_solver_node` |
| **Subscribes** | `/soa/sensor_geometry` |
| **Publishes** | `/soa/index` |

Constructs the observability matrix and evaluates the SOA index.

---

### `soa_visualizer`

| | |
|---|---|
| **Node name** | `soa_visualizer_node` |
| **Subscribes** | `/soa/index` |

Prints the current SOA index to the ROS console.

## License

MIT License
