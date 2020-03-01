/*
 * EulerAngles
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Stores roll, pitch, yaw.
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
#include "EulerAngles.hpp"

/**
 * Constructor for EulerAngles.
 */
EulerAngles::EulerAngles(): roll(0), pitch(0), yaw(0) {}

/**
 * Constructor for EulerAngles with initial values.
 */
EulerAngles::EulerAngles(double roll, double pitch, double yaw) :
    roll(roll), pitch(pitch), yaw(yaw) {}

/**
 * Converts EulerAngles to degrees.
 */
EulerAngles EulerAngles::toDegrees() {
    EulerAngles degAngles;
    degAngles.roll = roll * (180/PI);
    degAngles.pitch = pitch * (180/PI);
    degAngles.yaw = yaw * (180/PI);
    return degAngles;
}

/**
 * Converts EulerAngles to radians.
 */
EulerAngles EulerAngles::toRadians() {
    EulerAngles radAngles;
    radAngles.roll = roll * (PI/180.0);
    radAngles.pitch = pitch * (PI/180.0);
    radAngles.yaw = yaw * (PI/180.0);
    return radAngles;
}

/**
 * Returns string of euler angles in format: "ROLL,PITCH,YAW".
 */
String EulerAngles::toString() {
    return roll + "," + pitch + "," + yaw;
}

