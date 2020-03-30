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
INS::INS() {}

/**
 * Destructor for INS.
 */
INS::~INS() {}

/**
 * Event Handler for when received accelerometer data.
 */
void INS::onAccelerometerData(double x, double y, double z) {
    printf("acc: %f, %f, %f\n", x, y, z);
}

/**
 * Event Handler for when received gyroscope data.
 */
void INS::onGyroscopeData(double pitch, double roll, double yaw) {
    printf("gyro: %f, %f, %f\n", pitch, roll, yaw);
}

/**
 * Event Handler for when received magnetometer data.
 */
void INS::onMagnetometerData(double x, double y, double z) {}
