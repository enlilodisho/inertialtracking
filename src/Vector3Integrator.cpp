/*
 * Vector3 Integrator
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Uses trapezoidal rule to approximate the definite integral of a Vector3 obj.
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
#include "Vector3Integrator.hpp"

/**
 * Constructor for Vector3Integrator.
 */
Vector3Integrator::Vector3Integrator() {
    result = { x: 0.0, y: 0.0, z: 0.0 };
    prevData = { x: 0.0, y: 0.0, z: 0.0 };
}

/**
 * Destructor for Vector3Integrator.
 */
Vector3Integrator::~Vector3Integrator() {}

/**
 * Integrates new data.
 */
void Vector3Integrator::onNewData(Vector3<double> newData, double dt_ns) {
    // Get delta time in seconds.
    double dt_s = dt_ns / 1000000000;

    result.x += dt_s * ((prevData.x + newData.x) / 2.0);
    result.y += dt_s * ((prevData.y + newData.y) / 2.0);
    result.z += dt_s * ((prevData.z + newData.z) / 2.0);

    prevData.x = newData.x; prevData.y = newData.y; prevData.z = newData.z;
}

/**
 * Returns the integration.
 */
Vector3<double> Vector3Integrator::getResult() {
    return result;
}
