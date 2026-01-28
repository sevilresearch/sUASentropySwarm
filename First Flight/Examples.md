# EXAMPLES



## Hardware
- Crazyflie 2.x + battery
- Crazyradio 2.0
- Flow deck (recommended for stable hover indoors)

# Example 1 — Hover with Flow deck (Windows, non-ROS2)

This script connects to a Crazyflie via Crazyradio, enables the Kalman estimator + High-Level Commander,
logs `stateEstimate.x/y/z`, and performs:
- Takeoff to a user-defined altitude
- Hover with optional XY auto-hold (recenter toward takeoff point)
- Land

# Example 2 — Go to ABSOLUTE (x, y, z) with Flow deck (Windows, non-ROS2)

This GUI script connects to a Crazyflie via Crazyradio, enables Kalman + High-Level Commander,
logs `stateEstimate.x/y/z`, and moves the drone to a user-defined absolute setpoint (x,y,z)
in the estimator frame (meters).

It uses small motion segments (step-based) for safer, more robust tracking with Flow.

Example 3 — Swarm x2 “Square Mission” (Windows, non-ROS2)

This script connects to two Crazyflies via Crazyradio, enables the High-Level Commander, 
switches to the Kalman estimator (and Mellinger controller), resets the estimator, 
then commands both drones to fly a simple square pattern using relative go_to() steps, and finally lands both vehicles.
