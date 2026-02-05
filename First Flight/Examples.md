# EXAMPLES
These examples are a step-by-step onboarding path for flying Crazyflies and validating core capabilities before running the ROS2 entropy-swarm controller. They are intentionally simple, conservative, and designed to be repeatable. Start with single-drone hover, then single-drone waypoint motion, and finally two-drone coordinated flight. The scripts in first_flight/ run on Windows (non-ROS2) using Bitcraze’s Python tooling and are meant to confirm your hardware, estimator, and communication link are working reliably.

## Requeriments
### Hardware
- Crazyflie 2.x + battery
- Crazyradio 2.0
- Flow deck (for examples: 1-3)
- For Examples 4-6
    - OptiTrack motion-capture system (cameras + calibrated capture volume) running Motive
    - Rigid body markers/constellation mounted on the Crazyflie (or a rigid body tracked as the Crazyflie) with a consistent Rigid Body ID / name (e.g., CF1, CF2)
    - A network connection between the Motive PC and your script PC (same machine is OK) configured for NatNet streaming (your scripts assume multicast/Z-up style settings and specify server/local IPs + ports)

### Software
- Python 3 (Bitcraze’s Python tooling is built around cflib)
- pip for installing Python packages
- cflib (pip install cflib) — Bitcraze’s Crazyflie Python library used to communicate/control the Crazyflie
- For examples 4-6
    - Motive running and streaming rigid bodies via NatNet
    - A compatible NatNetClient Python module available in the same folder / Python path as your script (included in the files)

## Example 1 — Hover with Flow deck (Windows, non-ROS2)

This script connects to a Crazyflie via Crazyradio, enables the Kalman estimator + High-Level Commander,
logs `stateEstimate.x/y/z`, and performs:
- Takeoff to a user-defined altitude
- Hover with optional XY auto-hold (recenter toward takeoff point)
- Land

## Example 2 — Go to absolute (x, y, z) with Flow deck (Windows, non-ROS2)

This GUI script connects to a Crazyflie via Crazyradio, enables Kalman + High-Level Commander,
logs `stateEstimate.x/y/z`, and moves the drone to a user-defined absolute setpoint (x,y,z)
in the estimator frame (meters).

## Example 3 — Swarm x2 “Square Mission” (Windows, non-ROS2)

This script connects to two Crazyflies via Crazyradio, enables the High-Level Commander, 
switches to the Kalman estimator (and Mellinger controller), resets the estimator, 
then commands both drones to fly a simple square pattern using relative go_to() steps, and finally lands both vehicles.

## Example 4 — Hover using OptiTrack

This script connects to a Crazyflie via Crazyradio and uses OptiTrack/NatNet to get the drone’s live X/Y/Z position and yaw. It provides a GUI to take off, hover, and land, and includes an Auto-Hold mode that continuously recenters the drone back to its takeoff point using a smooth velocity (P) controller, rotating world-frame position error into the Crazyflie body frame using the OptiTrack yaw.

## Example 5 — Go to absolute (x, y, z) using OptiTrack (NatNet) + extpos

This script uses OptiTrack/NatNet to track a selected rigid body (by name or ID) and continuously feeds the Crazyflie’s external position via cf.extpos.send_extpos() at a high rate so the Kalman estimator can localize in the mocap frame. It provides a GUI to take off, command an absolute (x,y,z) target, step toward the target in bounded increments (step size + velocity + tolerance + timeout + Z limits), and optionally return to the takeoff XY (“home”). It also includes automatic CSV logging of position, target, and error from takeoff until landing/exit.

## Example 6 — Waypoint Navigation "Square Mission" (absolute XYZ + extpos + autonomous square)

This script uses OptiTrack/NatNet to track the Crazyflie pose and continuously feeds external position to the Crazyflie (extpos) so the Kalman estimator stays aligned with the mocap frame. It provides a GUI for manual takeoff/land and Go-To (x,y,z), and adds an autonomous square mission that automatically commands a sequence of absolute waypoints (a square of user-defined side length) and lands at the end. It also includes automatic CSV logging of position, target, and tracking error during the run.
