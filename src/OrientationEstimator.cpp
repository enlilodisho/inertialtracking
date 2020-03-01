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
 * Update estimate with accelerometer data.
 * Should be called whenever received accelerometer data.
 */
void OrientationEstimator::onAccelerometerData(Vector3 data) {
    // Can't use accelerometer for orientation for rockets.
}

/**
 * Update estimate with magnetometer data.
 * Should be called whenever received magnetometer data.
 */
void OrientationEstimator::onMagnetometerData(Vector3 data) {
    // TODO more research required to see if we can use magnetometer data
    // for orientation on rockets.
}

/**
 * Update estimate with gyroscope data.
 * Should be called whenever received gyroscope data.
 */
void OrientationEstimator::onGyroscopeData(Vector3 data) {
    // TODO
}

