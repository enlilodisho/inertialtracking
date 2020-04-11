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
#include <math.h>

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

/**
 * Quaternion division operator overload.
 */
Quaternion Quaternion::operator/(double k) {
    double newW = w / k;
    double newX = x / k;
    double newY = y / k;
    double newZ = z / k;
    Quaternion qR(newW, newX, newY, newZ);
    return qR;
}
Quaternion& Quaternion::operator/=(double k) {
    w /= k;
    x /= k;
    y /= k;
    z /= k;
    return *this;
}
Quaternion Quaternion::operator/(const Quaternion& q2) {
    double n2 = norm_squared();
    double newW = (w*q2.w + x*q2.x + y*q2.y + z*q2.z)  / n2;
    double newX = (-w*q2.x + x*q2.w - y*q2.z + z*q2.y) / n2;
    double newY = (-w*q2.y + x*q2.z + y*q2.w - z*q2.x) / n2;
    double newZ = (-w*q2.z - x*q2.y + y*q2.x + z*q2.w) / n2;
    Quaternion qR(newW, newX, newY, newZ);
    return qR;
}
Quaternion& Quaternion::operator/=(const Quaternion& q2) {
    double n2 = norm_squared();
    double newW = (w*q2.w + x*q2.x + y*q2.y + z*q2.z)  / n2;
    double newX = (-w*q2.x + x*q2.w - y*q2.z + z*q2.y) / n2;
    double newY = (-w*q2.y + x*q2.z + y*q2.w - z*q2.x) / n2;
    double newZ = (-w*q2.z - x*q2.y + y*q2.x + z*q2.w) / n2;
    w = newW; x = newX; y = newY; z = newZ;
    return *this;
}

/**
 * Returns the conjugate of this quaternion.
 */
Quaternion Quaternion::conjugate() {
    double newX = -x;
    double newY = -y;
    double newZ = -z;
    Quaternion qC(w, newX, newY, newZ);
    return qC;
}

/**
 * Returns the normalized quaternion.
 */
Quaternion Quaternion::normalize() {
    double norm = sqrt(norm_squared());
    double newW = w/norm;
    double newX = x/norm;
    double newY = y/norm;
    double newZ = z/norm;
    Quaternion qR(newW, newX, newY, newZ);
    return qR;
}

/**
 * Returns the square of the norm of the Quaternion.
 */
double Quaternion::norm_squared() {
    return w*w + x*x + y*y +z*z;
}

/**
 * Returns the inverse of this quaternion.
 */
Quaternion Quaternion::inverse() {
    return conjugate() / norm_squared();
}

/**
 * Calculate and Return euler angles for this quaternion.
 */
EulerAngles Quaternion::toEulerAngles() {
    struct EulerAngles angles;

    // roll
    double sinr_cosp = 2*(w*x + y*z);
    double cosr_cosp = 1 - 2*(x*x + y*y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch
    double sinp = 2*(w*y - z*x);
    if (abs(sinp) >= 1) {
        angles.pitch = copysign(M_PI / 2.0, sinp);
    } else {
        angles.pitch = asin(sinp);
    }

    // yaw
    double siny_cosp = 2*(w*z + x*y);
    double cosy_cosp = 1 - 2*(y*y + z*z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}
