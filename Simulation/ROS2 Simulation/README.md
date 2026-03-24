
# Running the Simulation (ROS 2 + Crazyswarm2)

This repository runs an entropy-based swarm controller for Crazyflie drones using ROS 2. It supports Crazyswarm2 simulation and logs run data to a CSV file, then generates plots.

### Prerequisites
  - OS + ROS 2
  
  - Ubuntu 22.04
  
  - ROS 2 Humble installed and sourced

### Packages you must have
  - Crazyswarm2 (simulation)
  
  - Crazyflie ROS 2 interfaces (crazyflie_interfaces)
  
  - TF2 (tf2_ros)

If you already have Crazyswarm2 working, you’re good.

## 1. Start the Crazyflie simulation (Crazyswarm2)

**In a new terminal:**

   source /opt/ros/humble/setup.bash
   
   source ~/ros2_ws/install/setup.bash
   
   ros2 launch crazyflie launch.py backend:=sim

### Verify simulation is up

**In another terminal:**

   ros2 service list | grep -i takeoff

   ros2 topic list | grep /tf

You should see CF services (e.g., /CF1/takeoff, /CF2/go_to, etc.) and /tf.

## 2. Run the entropy swarm node

### Open a new terminal:

   source /opt/ros/humble/setup.bash

   source ~/ros2_ws/install/setup.bash


**Option A (recommended): run as a ROS 2 node**


If your package installs an executable entrypoint:

ros2 run entropy_swarm entropy_swarm_node


**Option B: run directly with Python (if you’re iterating)**


   python3 ~/ros2_ws/src/entropy_swarm/entropy_swarm/entropy_swarm_node.py


## 3. Recommended “first run” settings

For a stable first simulation:

   - dmin = 0.30
   
   - dmax = 1.50
   
   - z = 0.50
   
   - threshold = 0.50

   - closeEnough = 0.30 

   - EPS = 0.05 (repel earlier = safer)
