/*
 * Quaternion
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Class for doing quaternion math.
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
#include "Quaternion.hpp"

/**
 * Constructor 1.
 */
Quaternion::Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

/**
 * Constructor 2.
 */
Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

/**
 * Quaternion multiplication operator overload.
 */
Quaternion Quaternion::operator*(const Quaternion& q2) {
    double newW = w*q2.w - x*q2.x - y*q2.y - z*q2.z;
    double newX = w*q2.x + x*q2.w + y*q2.z - z*q2.y;
    double newY = w*q2.y - x*q2.z + y*q2.w + z*q2.x;
    double newZ = w*q2.z + x*q2.y - y*q2.x + z*q2.w;
    Quaternion qR(newW, newX, newY, newZ);
    return qR;
}
Quaternion& Quaternion::operator*=(const Quaternion& q2) {
    double newW = w*q2.w - x*q2.x - y*q2.y - z*q2.z;
    double newX = w*q2.x + x*q2.w + y*q2.z - z*q2.y;
    double newY = w*q2.y - x*q2.z + y*q2.w + z*q2.x;
    double newZ = w*q2.z + x*q2.y - y*q2.x + z*q2.w;
    w = newW; x = newX; y = newY; z = newZ;
    return *this;
}
