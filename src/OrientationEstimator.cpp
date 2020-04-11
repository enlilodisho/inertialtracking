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
#include <iomanip>

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
    //printf("%f,%f,%f\n", pitch, roll, yaw);
    // Get delta time in seconds.
    double dt_s = dt_ns / 1000000000;

    // Perform Gyro Integration using Quaternions //
    
    // Get magnitude of gyro data.
    double gyroMag = sqrt(pitch*pitch + roll*roll + yaw*yaw);
    if (gyroMag < 0.3) {
        // No movement detected.
        return;
    }

    ///////////////////////////////
    // Old method using euler angles:
    /*
    static struct EulerAngles prevAngles;
    static struct EulerAngles eulerOrientation;
    struct EulerAngles rotAngles;
    rotAngles.pitch = ((pitch + prevAngles.pitch) / 2.0) * dt_s;
    rotAngles.roll  = ((roll  + prevAngles.roll ) / 2.0) * dt_s;
    rotAngles.yaw   = ((yaw   + prevAngles.yaw  ) / 2.0) * dt_s;
    //printf("Rot angles: pitch=%f, roll%f, yaw=%f\n", rotAngles.pitch,
    //        rotAngles.roll, rotAngles.yaw);
    
    eulerOrientation.pitch += rotAngles.pitch;
    eulerOrientation.roll  += rotAngles.roll;
    eulerOrientation.yaw   += rotAngles.yaw;
    printf("euler: pitch=%f, roll=%f, yaw=%f\n",
            eulerOrientation.pitch, eulerOrientation.roll,
            eulerOrientation.yaw);
    */
    ///////////////////////////////
    
    // Normalize gyro data.
    double normPitch = pitch / gyroMag;
    double normRoll  = roll / gyroMag;
    double normYaw   = yaw / gyroMag;
    
    // Convert mesasurements to instantaneous rotation quaternion
    double theta = gyroMag * dt_s * (M_PI / (180.0*2));
    /*printf("normPitch=%f, normRoll=%f, normYaw=%f, theta=%f\n", normPitch,
            normRoll, normYaw, theta);*/
    Quaternion rotQuat(cos(theta/2.0), normRoll*sin(theta/2.0),
            normPitch*sin(theta/2.0), normYaw*sin(theta/2.0));
    /*printf("Rot quat: w=%f, x=%f, y=%f, z=%f\n", rotQuat.w, rotQuat.x,
            rotQuat.y, rotQuat.z);*/

    // Integrate
    orientation *= rotQuat;

    /*printf("quat: w=%f, x=%f, y=%f, z=%f\n", orientation.w,
            orientation.x, orientation.y, orientation.z);*/

    /*
    struct EulerAngles orientationAngles = orientation.toEulerAngles();
    // convert to degrees
    orientationAngles.pitch *= 180/M_PI;
    orientationAngles.roll  *= 180/M_PI;
    orientationAngles.yaw   *= 180/M_PI;
    printf("quat in euler: pitch=%f, roll=%f, yaw=%f\n",
            orientationAngles.pitch, orientationAngles.roll,
            orientationAngles.yaw);
    printf("\n");*/
}

/**
 * Update estimate with magnetometer data.
 * Should be called whenever received magnetometer data.
 */
void OrientationEstimator::onMagnetometerData(double x, double y, double z, double dt_ns) {
    // TODO more research required to see if we can use magnetometer data
    // for orientation on rockets.
}

