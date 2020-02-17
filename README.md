# Inertial Tracking for Arduino (WIP)
- Using a LSM9DS1 module.

**NOTICE! This is an experiment and not something that you can easily implement in your project.**

---------------------------------------

### Introduction

This is an experiment to attempt to implement Inertial Tracking using cheap MEMS sensors. These sensors are not designed for this use-case and so the position calculated from them is often too noisy for most projects. For me, I do not need very high accuracy position data, but only the altitude of the device. As part of the Rocket Propulsion Laboratory (RPL) at UCSD, we are planning to use the LSM9DS1 sensor combined with GPS to determine the maximum height of our rocket. With this experiment, I am trying to see whether this would be possibile using such cheap accelerometer/gyroscope/magnetometer sensors.


### Main Idea

Accelerometers give us acceleration.
Gyroscopes give us rotational velocity.
Without considering noise, we should get position from accelerometers and gyroscopes as follows:

1. We first have to find the orientation of the device using a gyroscope.

    - Gyroscope data is very accurate although it suffers from drift. To remove this drift, it is typically done by also finding orientation using an accelerometer and then combining the two orientations using some filter. For our use-case, we are not able to do this since the rocket will be accelerating upwards and so we won't be able to find orientation using an accelerometer (the accelerometer won't be able to measure the gravity vector). Thus, we have to find another way to solve this problem.

2. Using the orientation of the device, we rotate the accelerometer data from the sensor-frame to the world-frame. This is so that acceleration in the Z direction will always be relative to the surface of the earth rather than the sensor.

3. Now we subtract gravity from the accelerometer data so that we get linear acceleration.

4. Finally we do a double integral on linear acceleration to get relative position.


### License

This code is licensed under the **GNU General Public License v3.0 (GNU GPLv3).**

Basically, <u>you must share any modifications you make to this code</u>, where a modification means "to copy from or adapt all or part of the work in a fashion requiring copyright permission, other than the making of an exact copy".

See LICENSE.md for full license.
