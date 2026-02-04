# First flight (Crazyflie 2.x) 

This project assumes you can perform a safe manual first flight before running any ROS2 autonomy. 

The steps below summarize Bitcraze’s official “Getting started with Crazyflie 2.0/2.1(+)” tutorial. https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#inst-comp

## 1) Pre-flight checks 

Run the power-on self-test (before assembly changes). Bitcraze recommends powering the Crazyflie from USB and checking the LED pattern; a fast green blink indicates the self-test passed.

On power-up the Crazyflie runs self-tests and calibrates sensors; it should be absolutely still on a level surface during calibration.

Orientation: the small antenna “bump” is the front and the blue LEDs are at the back (useful for first flights).

## 2) Choose how you will control it (PC recommended)

You can fly from mobile or computer, but recommends a PC + Crazyradio.

## 3) Install and start the Crazyflie client (computer path)

Install/run the Crazyflie client via pip (native install recommended), then plug in the Crazyradio and your game controller, and start the client.


## 4) Configure the client (controller + firmware + connect)

In the Crazyflie client:

Open input device settings and confirm the correct controller mapping/device type.

Update the Crazyflie firmware to the newest version.

Connect by selecting the address (default shown in the tutorial), scan, choose your Crazyflie, and connect; once connected you should see telemetry (battery and link quality updating live).

## 5) First flight tips (what beginners usually struggle with)

Bitcraze’s basic flying notes that help most on day 1:

- Start with the Crazyflie pointing away from you (blue LEDs toward you).

- Learn the four control dimensions: roll, pitch, yaw, thrust.

- Expect some drift in normal flight—Bitcraze notes this can be normal and needs compensation.

- Ground effect: close to the ground the airflow can feel “slippery”; Bitcraze suggests using more thrust right at takeoff, then easing off to level out.

- If it drifts a lot at takeoff, Bitcraze suggests checking: battery centered, props spin freely (hair/debris), and prop balance.

## 6) Charging

Charging by plugging in micro-USB while powered on; the back left blue LED blinks while charging and becomes fully lit when charged.
