/*
 * Accelerometer Data Transformer
 * Copyright (C) 2020  Enlil Odisho
 * ------------------------------------------------
 * Removes gravity from accelerometer.
 * Transforms data from the sensor frame to the world frame.
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
#include "AccelerometerDataTransformer.hpp"

/**
 * Constructor for AccelerometerDataTransformer.
 */
AccelerometerDataTransformer::AccelerometerDataTransformer() {
  const double GRAVITY_FORCE = 9.80665;  // [TODO] Gravity changes with altitude
  gravity = new Vector3(0, 0, GRAVITY_FORCE);
}

/**
 * Destructor for AccelerometerDataTransformer.
 */
AccelerometerDataTransformer::~AccelerometerDataTransformer() {
  delete gravity;
}

/**
 * Updates the orientation of device.
 */
void AccelerometerDataTransformer::updateOrientation(EulerAngles orientation) {
  // Get magnitude of gravity (typically ~9.81)
  double gravityMag = gravity->getMagnitude();

  this->orientation = orientation;
  // Convert orientation to radians
  orientation = orientation.toRadians();

  // Transform gravity vector
  tGravity.x = -gravityMag*sin(orientation.yaw);
  tGravity.y = gravityMag*cos(orientation.yaw)*sin(orientation.roll);
  tGravity.z = gravityMag*cos(orientation.yaw)*cos(orientation.roll);
}

/**
 * Removes gravity from acceleration.
 */
void AccelerometerDataTransformer::subtractGravity(Vector3 *data) {
  data->x -= tGravity.x;
  data->y -= tGravity.y;
  data->z -= tGravity.z;
}

/**
 * Removes acceleration vector from sensor to world frame.
 */
void AccelerometerDataTransformer::rotateToWorldFrame(Vector3 *data) {
  //Serial.print(orientation.roll);Serial.print(",");Serial.print(orientation.pitch);Serial.print(",");Serial.print(orientation.yaw);Serial.print(",");
  EulerAngles o = orientation.toRadians();
  double roll = o.roll;
  double pitch = o.pitch;
  double yaw = o.yaw;

  /* Online algorithm from stack overflow */
  /* Gets unit vector of rotation */
  /*data->x = cos(yaw)*cos(pitch);
  data->y = sin(yaw)*cos(pitch);
  data->z = sin(pitch);
  Vector3 yDirRot;
  yDirRot.x = -cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
  yDirRot.y = -sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
  yDirRot.z = cos(pitch)*sin(roll);*/
  data->x = -cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll);
  data->y = -sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll);
  data->z = cos(pitch)*sin(roll);

  /* Matt's algorithm */
  /*data->x = data->y*cos(roll)*sin(yaw)+(data->x*sin(pitch)+data->z*cos(pitch))*sin(roll)*sin(yaw)+(data->x*cos(pitch)-data->z*sin(pitch))*cos(yaw);
  data->y = data->y*cos(roll)*cos(yaw)+(data->x*sin(pitch)+data->z*cos(pitch))*sin(roll)*cos(yaw)+(data->z*sin(pitch)-data->x*cos(pitch))*sin(yaw);
  data->z = (data->x*sin(pitch)+data->z*cos(pitch))*cos(roll)-data->y*sin(roll);*/
  /*//Serial.print(data->x);Serial.print(",");
  if (orientation.pitch >= -90 && orientation.pitch <= 90) {
    EulerAngles radO = orientation.toRadians();
    
    //data->x *= cos(radO.pitch);
    data->x = data->x*cos(radO.pitch) - data->z*sin(radO.pitch);
    //data->x = data->x*cos(PI/2.0) - data->z*sin(PI/2.0);

    data->y = data->y*cos(radO.roll) - data->z*sin(radO.roll);
  }*/

  /*Serial.print(orientation.pitch);Serial.print(",");
  Serial.println(data->x);*/
  //Serial.print(data->x);Serial.print(",");Serial.println(data->y);
  //Serial.println(orientation.pitch);*/
  
  /*double theta = orientation.yaw; 
  double beta = orientation.roll;
  double alpha = orientation.pitch;
  data->x = data->x*cos(theta)*cos(beta) + data->x*sin(theta)*cos(beta) - data->x*sin(beta);
  data->y = data->y*(-sin(theta)*cos(alpha)+cos(theta)*sin(beta)*sin(alpha)) + data->y*(cos(theta)*cos(alpha)+sin(theta)*sin(beta)*sin(alpha)) + data->y*(cos(beta)*sin(alpha));
  data->z = data->z*(sin(theta)*sin(alpha)+cos(theta)*sin(beta)*cos(alpha)) - data->z*(-cos(theta)*sin(alpha)+sin(theta)*sin(beta)*cos(alpha)) + data->z*(cos(beta)*cos(alpha));*/
}
