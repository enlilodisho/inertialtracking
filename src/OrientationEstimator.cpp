/*
 * Orientation Estimator
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Estimates the orientation of the device.
 * 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "OrientationEstimator.hpp"
#include <math.h>
#include <iostream>

/**
 * Constructor for OrientationEstimator.
 */
OrientationEstimator::OrientationEstimator() {}

/**
 * Destructor for OrientationEstimator.
 */
OrientationEstimator::~OrientationEstimator() {}

/**
 * Returns the current estimated orientation expressed as a quaternion.
 */
Quaternion OrientationEstimator::getOrientation() {
    return orientation;
}

/**
 * Update estimate with gyroscope data.
 * Should be called whenever received gyroscope data.
 */
void OrientationEstimator::onGyroscopeData(double pitch, double roll, double yaw, double dt_ns) {
    // Get delta time in seconds.
    double dt_s = dt_ns / 1000000000;

    // Perform Gyro Integration using Quaternions //
    
    // Get magnitude of gyro data.
    double gyroMag = sqrt(pitch*pitch + roll*roll + yaw*yaw);
    if (gyroMag == 0) {
        // No movement detected.
        return;
    }
    // Normalize gyro data.
    double normPitch = pitch / gyroMag;
    double normRoll  = roll / gyroMag;
    double normYaw   = yaw / gyroMag;
    
    // Convert mesasurements to instantaneous rotation quaternion
    double theta = gyroMag * dt_s;
    Quaternion rotQuat(cos(theta/2.0), normPitch*sin(theta/2.0),
            normRoll*sin(theta/2.0), normYaw*sin(theta/2.0));

    // Integrate
    orientation *= rotQuat;

    printf("Orientation quat: w=%f, x=%f, y=%f, z=%f\n", orientation.w,
            orientation.x, orientation.y, orientation.z);
}

/**
 * Update estimate with magnetometer data.
 * Should be called whenever received magnetometer data.
 */
void OrientationEstimator::onMagnetometerData(double x, double y, double z, double dt_ns) {
    // TODO more research required to see if we can use magnetometer data
    // for orientation on rockets.
}

