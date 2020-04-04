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

/**
 * Constructor for INS.
 */
INS::INS() : active(false) {}

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
void INS::onMagnetometerData(double x, double y, double z, double dt_ns) {}
