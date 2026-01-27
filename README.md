# Small Unmanned Aircraft Systems Entropy Swarm
This repository contains an entropy-based distributed behavior controller for a small swarm of Crazyflie 2.1 nano-UAVs in **ROS 2**, using the **OptiTrack → ROS2 → Crazyswarm2 → Crazyflie** pipeline.

This repository focuses on implementing a **distributed switching behavior**: at each control step, each drone decides whether to:
- **GROUP** (cohere toward neighbors / local center of mass), or
- **MISSION** (move toward its assigned waypoint / lane goal),

based on a **Tsallis-entropy** measure and a **switching threshold with hysteresis**.

## Citation 
This project is based on entropy swarm behavior modeling from:
Molina, Juan Jose, "Small Unmanned Aircraft Systems Entropy Swarm", UWF, expected 2026.

## Related work
Fina et al., “Entropy-Based Distributed Behavior Modeling for Multi-Agent UAVs”, Drones, 2022.
(Also see sevilresearch/EntropySwarm for reference materials.)


## System overview (high level)

**Data flow**
1. OptiTrack Motive streams rigid-body poses (one per CF)
2. NatNet → ROS 2 bridge publishes extpos / TF at fixed rate
3. Crazyswarm2 feeds external position to Crazyflie high-level commander
4. `entropy_controller` computes distances + entropy and commands motion:
   - GROUP: move toward neighbor centroid (with spacing + repel)
   - MISSION: move toward goal waypoint / lane
   - REPEL: override when too close

**Modes**
- **GROUP**: encourages cohesion while enforcing minimum separation
- **MISSION**: goal-seeking toward assigned lane/waypoint
- **REPEL**: immediate override when min separation is violated

## Experiments included / intended

This controller is designed to support parameter sweeps such as:
1. **Threshold sweep** (`ENTHRESH`) → cohesion vs decisiveness trade-off
2. **Speed sweep** (`CRUISE_V`) → time vs smoothness/stability trade-off
3. **Distance window sweep** (`D_MIN`, `D_MAX`) → spacing policy effects
4. **Tsallis q sweep** (`Q_TSALLIS`) → switching sharpness + symmetry effects

---

## Requirements

- ROS 2 (recommended: Humble)
- Crazyswarm2 (simulation) or Crazyflie hardware stack (real flights)
- Python 3 + ROS 2 Python dependencies (`rclpy`, `tf2_ros`, `geometry_msgs`, `crazyflie_interfaces`)


## Quickstart 


### 1) Create / enter workspace
mkdir -p ros2_ws/src
cd ros2_ws/src

### 2) Clone this repo (or copy packages into src/)
git clone https://github.com/sevilresearch/sUASentropySwarm.git

### 3) Install dependencies
cd ..
rosdep install --from-paths src -y --ignore-src

### 4) Build
colcon build --symlink-install
source install/setup.bash

### 5) Run (example)
ros2 run <your_package_name> <your_node_executable>
