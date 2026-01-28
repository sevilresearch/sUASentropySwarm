# EXAMPLES
These examples are a step-by-step onboarding path for flying Crazyflies and validating core capabilities before running the ROS2 entropy-swarm controller. They are intentionally simple, conservative, and designed to be repeatable. Start with single-drone hover, then single-drone waypoint motion, and finally two-drone coordinated flight. The scripts in first_flight/ run on Windows (non-ROS2) using Bitcraze’s Python tooling and are meant to confirm your hardware, estimator, and communication link are working reliably.

## Requeriments
### Hardware
- Crazyflie 2.x + battery
- Crazyradio 2.0
- Flow deck (recommended for stable hover indoors)

### Software
- Python 3 (Bitcraze’s Python tooling is built around cflib)
- pip for installing Python packages
- cflib (pip install cflib) — Bitcraze’s Crazyflie Python library used to communicate/control the Crazyflie

## Example 1 — Hover with Flow deck (Windows, non-ROS2)

This script connects to a Crazyflie via Crazyradio, enables the Kalman estimator + High-Level Commander,
logs `stateEstimate.x/y/z`, and performs:
- Takeoff to a user-defined altitude
- Hover with optional XY auto-hold (recenter toward takeoff point)
- Land

## Example 2 — Go to ABSOLUTE (x, y, z) with Flow deck (Windows, non-ROS2)

This GUI script connects to a Crazyflie via Crazyradio, enables Kalman + High-Level Commander,
logs `stateEstimate.x/y/z`, and moves the drone to a user-defined absolute setpoint (x,y,z)
in the estimator frame (meters).

It uses small motion segments (step-based) for safer, more robust tracking with Flow.

## Example 3 — Swarm x2 “Square Mission” (Windows, non-ROS2)

This script connects to two Crazyflies via Crazyradio, enables the High-Level Commander, 
switches to the Kalman estimator (and Mellinger controller), resets the estimator, 
then commands both drones to fly a simple square pattern using relative go_to() steps, and finally lands both vehicles.
