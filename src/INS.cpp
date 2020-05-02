/*
 * Inertial Navigation System (INS)
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Tracks position of object based on data from inertial sensors.
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
#include <iostream>
#include "INS.hpp"
#include <math.h> //temp

#define LINEAR_ACC_CALIB_AMT    1000

/**
 * Constructor for INS.
 */
INS::INS() : active(false) {
    lacc_calib_counter = 0;
    lacc_calibration = { x: 0.0, y: 0.0, z: 0.0 };
}

/**
 * Destructor for INS.
 */
INS::~INS() {
    active = false;
    std::cout << "DESTROYED INS\n";
}

/**
 * Start the INS thread.
 */
void INS::start() {
    active = true;
    std::thread insThread(&INS::loop, this);
    insThread.detach();
}

/**
 * Dequeue and process data.
 */
void INS::loop() {
    while (true) {
        if (active == false) {
            break;
        }

        queueMtx.lock();

        // Check queue sizes.
        int gyroQueueSize = gyroQueue.size();
        int accQueueSize = accQueue.size();
        if (gyroQueueSize == 0 || (gyroQueueSize == 1 && accQueueSize == 0)) {
            queueMtx.unlock();
            // Queue empty or data has not fully been received yet.
            if (active == true) {
                continue;
            }
            break;
        }
        //printf("GYRO.size = %d ; ACC.size = %d\n", gyroQueueSize, accQueueSize);

        struct QueueNode* gyroNode = gyroQueue.front();
        gyroQueue.pop_front();
        struct QueueNode* accNode = accQueue.front();
        accQueue.pop_front();

        queueMtx.unlock();

        // Update orientation estimator.
        orientationEstimator.onGyroscopeData(gyroNode->x, gyroNode->y,
                gyroNode->z, gyroNode->dt_ns);

        // Get the orientation of the device.
        Quaternion orientation = orientationEstimator.getOrientation();
        /*
        printf("Orientation: %f,%f,%f,%f", orientation.w, orientation.x, orientation.y, orientation.z);
        struct EulerAngles orientationAngles = orientation.toEulerAngles();
        orientationAngles.pitch *= 180/M_PI;
        orientationAngles.roll  *= 180/M_PI;
        orientationAngles.yaw   *= 180/M_PI;
        printf(" (%f,%f,%f)", orientationAngles.pitch, orientationAngles.roll, orientationAngles.yaw);
        printf("\n");
        */

        // Transform acceleration from sensor-frame to world-frame.
        Quaternion qAcc(0, accNode->y, accNode->x, accNode->z);
        Quaternion qAccWorld = orientation*qAcc*orientation.inverse();
        //printf("Acc: %f,%f,%f,%f -> ", qAcc.w, qAcc.x, qAcc.y, qAcc.z);
        //printf("%f,%f,%f,%f\n", qAccWorld.w, qAccWorld.x, qAccWorld.y, qAccWorld.z);
        Vector3<double> worldAcc = { x: qAccWorld.y, y: qAccWorld.x,
            z: qAccWorld.z };


        // Do calibration for linear acceleration (if haven't done so yet)
        if (lacc_calib_counter <= LINEAR_ACC_CALIB_AMT) {
            if (lacc_calib_counter == 0) {
                printf("[INS] DO NOT MOVE DEVICE! Calibrating linear acc...\n");
            }

            if (lacc_calib_counter == LINEAR_ACC_CALIB_AMT) {
                lacc_calibration.x /= LINEAR_ACC_CALIB_AMT;
                lacc_calibration.y /= LINEAR_ACC_CALIB_AMT;
                lacc_calibration.z /= LINEAR_ACC_CALIB_AMT;
                printf("[INS] Linear acc calibration result: %f,%f,%f\n",
                        lacc_calibration.x, lacc_calibration.y,
                        lacc_calibration.z);
            } else {
                lacc_calibration.x += worldAcc.x;
                lacc_calibration.y += worldAcc.y;
                lacc_calibration.z += worldAcc.z;
            }
            lacc_calib_counter++;

            delete gyroNode;
            delete accNode;
            continue;
        }

        // Get linear acceleration in world-frame.
        Vector3<double> linearAcc;
        linearAcc.x = worldAcc.x - lacc_calibration.x;
        linearAcc.y = worldAcc.y - lacc_calibration.y;
        linearAcc.z = worldAcc.z - lacc_calibration.z;
        //printf("LACC: %f,%f,%f\n", linearAcc.x, linearAcc.y, linearAcc.z);

        // Get velocity by integrating lacc.
        laccIntegrator.onNewData(linearAcc, accNode->dt_ns);
        Vector3<double> velocity = laccIntegrator.getResult();
        //printf("Velocity: %f,%f,%f\n", velocity.x, velocity.y, velocity.z);

        // Get position by integrating velocity.
        velIntegrator.onNewData(velocity, accNode->dt_ns);
        Vector3<double> position = velIntegrator.getResult();
        //printf("Position: %f,%f,%f\n", position.x, position.y, position.z);

        //static int i = 0;
        //printf("%d,%f,%f,%f\n", (++i), linearAcc.z, velocity.z, position.z);

        delete gyroNode;
        delete accNode;
    }
}

/**
 * Event Handler for when received accelerometer data.
 */
void INS::onAccelerometerData(double x, double y, double z, double dt_ns) {
    //printf("acc: %f, %f, %f, dt_ns: %f\n", x, y, z, dt_ns);

    struct QueueNode* accNode = new struct QueueNode;
    accNode->x = x;
    accNode->y = y;
    accNode->z = z;
    accNode->dt_ns = dt_ns;

    queueMtx.lock();
    accQueue.push_back(accNode);
    queueMtx.unlock();
}

/**
 * Event Handler for when received gyroscope data.
 */
void INS::onGyroscopeData(double pitch, double roll, double yaw, double dt_ns) {
    //printf("gyro: %f, %f, %f, dt_ns: %f\n", pitch, roll, yaw, dt_ns);

    struct QueueNode* gyroNode = new struct QueueNode;
    gyroNode->x = pitch;
    gyroNode->y = roll;
    gyroNode->z = yaw;
    gyroNode->dt_ns = dt_ns;

    queueMtx.lock();
    gyroQueue.push_back(gyroNode);
    queueMtx.unlock();
}

/**
 * Event Handler for when received magnetometer data.
 */
void INS::onMagnetometerData(double x, double y, double z, double dt_ns) {
    printf("%f,%f,%f\n", x, y, z);
}
