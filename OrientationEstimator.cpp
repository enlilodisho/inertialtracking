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
#include "EulerAngles.hpp"
#include "Vector3Calibrator.hpp"
#include <Arduino.h>
#include <math.h>

/**
   Constructor for OrientationEstimator.
*/
OrientationEstimator::OrientationEstimator() {
}

/**
   Destructor for OrientationEstimator.
*/
OrientationEstimator::~OrientationEstimator() {

}

/**
   Update estimate with accelerometer data.
   Should be called whenever received accelerometer data.
*/
void OrientationEstimator::onAccelerometerData(Vector3 data) {
  unsigned long timeCurrent = micros(); // get current time in us
  static unsigned long timeLast = 0; // time of last reading
  if (timeLast == 0) {
    // This is first time receiving accelerometer reading. Update time, and wait for next reading.
    timeLast = timeCurrent;
    return;
  }
  // Calculate the time b/n this and the last accelerometer reading.
  int32_t dt_us = timeCurrent - timeLast;
  double dt_s = dt_us / 1000000.0;

  double roll = 180 * atan2(data.y, sqrt(data.x*data.x + data.z*data.z)) / PI;
  double pitch = 180 * atan2(data.x, sqrt(data.y*data.y + data.z*data.z)) / PI;

  // Update rotation based on this reading.
  accMagEulerOrientation.roll = roll;
  accMagEulerOrientation.pitch = pitch;
  // note: yaw cannot be determined using accelerometer.
  //Serial.println(accMagEulerOrientation.toString());

  // Update the time of last reading to current, for next time we call this method.
  timeLast = timeCurrent;
}

/**
   Update estimate with magnetometer data.
   Should be called whenever received magnetometer data.
*/
void OrientationEstimator::onMagnetometerData(Vector3 data) {
  unsigned long timeCurrent = micros(); // get current time in us
  static unsigned long timeLast = 0; // time of last reading
  if (timeLast == 0) {
    // This is first time receiving magnetometer reading. Update time, and wait for next reading.
    timeLast = timeCurrent;
    return;
  }
  // Calculate the time b/n this and the last magnetometer reading.
  int32_t dt_us = timeCurrent - timeLast;
  double dt_s = dt_us / 1000000.0;

  EulerAngles o = getOrientationInEulerAngles();
  // Math from: http://students.iitk.ac.in/roboclub/2017/12/21/Beginners-Guide-to-IMU.html
  double x = data.x*cos(o.pitch) + data.y*sin(o.roll)*sin(o.pitch) + data.z*cos(o.roll)*sin(o.pitch);
  double y = data.y*cos(o.roll) - data.z*sin(o.roll);
  //accMagEulerOrientation.yaw = 180 * atan2(-y,x) / PI; // TODO not working
  //Serial.println(accMagEulerOrientation.toString());

  // Update the time of last reading to current, for next time we call this method.
  timeLast = timeCurrent;
}

/**
   Update estimate with gyroscope data.
   Should be called whenever received gyroscope data.
*/
void OrientationEstimator::onGyroscopeData(Vector3 data) {
  unsigned long timeCurrent = micros(); // get current time in us
  static unsigned long timeLast = 0; // time of last reading
  // This is first time receiving gyroscope reading. Update time, and wait for next reading.
  if (timeLast == 0) {
    timeLast = timeCurrent;
    return;
  }
  // Calculate the time b/n this and the last gyroscope reading.
  int32_t dt_us = timeCurrent - timeLast;
  double dt_s = dt_us / 1000000.0;

  // The previous gyroscope reading from gyro
  static Vector3 prevGyro;
  
  EulerAngles rotationAngles;
  /*rotationAngles.roll = ((data.x + prevGyro.x) / 2.0) * dt_s  * (PI / 180.0);
  rotationAngles.pitch = ((data.y + prevGyro.y) / 2.0) * dt_s  * (PI / 180.0);
  rotationAngles.yaw = ((data.z + prevGyro.z) / 2.0) * dt_s  * (PI / 180.0);*/
  rotationAngles.roll = ((data.x + prevGyro.x) / 2.0) * dt_s;
  rotationAngles.pitch = ((data.y + prevGyro.y) / 2.0) * dt_s;
  rotationAngles.yaw = ((data.z + prevGyro.z) / 2.0) * dt_s;
  
  static Vector3Calibrator gyroCalibrator;
  static Vector3Calibrator rotationLinearDriftCalibrator; // averages linear slope approximations
  static int calibrateCount = 0;
  // Calibrate gyro drift.
  if (calibrateCount < 100) {
    gyroCalibrator.calibrate(rotationAngles);
    // Calculate slope
    EulerAngles rotationLinearDriftOffset;
    rotationLinearDriftOffset.yaw = rotationAngles.yaw / dt_s;
    rotationLinearDriftOffset.pitch = rotationAngles.pitch / dt_s;
    rotationLinearDriftOffset.roll = rotationAngles.roll / dt_s;
    rotationLinearDriftCalibrator.calibrate(rotationLinearDriftOffset);
    calibrateCount++;
  } else {
    // Offset gyroscope data.
    //gyroCalibrator.apply(&rotationAngles);

    Quaternion quatRotationAngles = Quaternion::from_euler_rotation(rotationAngles.toRadians());
    //quatOrientation = quatOrientation.rotate(quatRotationAngles.normalize());
    quatOrientation *= quatRotationAngles;

    gyroEulerOrientation.yaw += rotationAngles.yaw;
    gyroEulerOrientation.pitch += rotationAngles.pitch;
    gyroEulerOrientation.roll += rotationAngles.roll;

    //Serial.print(",");Serial.print(gyroEulerOrientation.roll);Serial.print(",");Serial.print(gyroEulerOrientation.pitch);Serial.print(",");Serial.println(gyroEulerOrientation.yaw);
    
    //rotationLinearDriftCalibrator.apply(&gyroEulerOrientation);

    /* Convert to radians
    rotationAngles.roll *= (PI/180.0);
    rotationAngles.pitch *= (PI/180.0);
    rotationAngles.yaw *= (PI/180.0);*/

    gyroEulerOrientation.yaw = fmod(gyroEulerOrientation.yaw, 360);
    gyroEulerOrientation.pitch = fmod(gyroEulerOrientation.pitch, 360);
    gyroEulerOrientation.roll = fmod(gyroEulerOrientation.roll, 360);
    /*Serial.print(timeCurrent);
    Serial.print(",");*/
    //Serial.println(gyroEulerOrientation.toString());
  }
  
  // Update previous gyroscope values for next time
  prevGyro.x = data.x;
  prevGyro.y = data.y;
  prevGyro.z = data.z;

  // Update the time of last reading to current, for next time we call this method.
  timeLast = timeCurrent;
}

/**
 * Fuses orientation from accelerometer, gyroscope, and magnetometer together
 * using a complimentary filter.
 */
void OrientationEstimator::fuse() {
  /*const float ALPHA = 0.01; // The percentage of acc data to use (smaller values = less noise, but more drift)
  orientation.roll = ALPHA * accMagEulerOrientation.roll + (1 - ALPHA) * gyroEulerOrientation.roll;
  orientation.pitch = ALPHA * accMagEulerOrientation.pitch + (1 - ALPHA) * gyroEulerOrientation.pitch;
  //orientation.yaw = gyroEulerOrientation.yaw;

  gyroEulerOrientation.roll = orientation.roll;
  gyroEulerOrientation.pitch = orientation.pitch;
  gyroEulerOrientation.yaw = orientation.yaw;*/

  orientation.roll = gyroEulerOrientation.roll;
  orientation.pitch = gyroEulerOrientation.pitch;
  orientation.yaw = gyroEulerOrientation.yaw;
  
  /*const float ALPHA = 0.01; // percentage of new value to use
  orientation.roll = ALPHA * gyroEulerOrientation.roll + (1 - ALPHA) * orientation.roll;
  orientation.pitch = ALPHA * gyroEulerOrientation.pitch + (1 - ALPHA) * orientation.pitch;
  orientation.yaw = ALPHA * gyroEulerOrientation.yaw + (1 - ALPHA) * orientation.yaw;*/
}

EulerAngles OrientationEstimator::getOrientationInEulerAngles() {
  return orientation;
  //return quatOrientation.toEulerAngles();
}
