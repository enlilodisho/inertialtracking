/*
 * InertialTracking-arduino
 * Copyright (C) 2020  Enlil Odisho
 * ------------------------------------------------
 * Arduino project. Calculates orientation, acceleration, and relative position
 * using a LSM9DS1 sensor.
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
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include "PrintEx.h"

#include "Vector3.hpp"
#include "OrientationEstimator.hpp"
#include "AccelerometerDataTransformer.hpp"
#include "Vector3Calibrator.hpp" //temp

//#define PRINT_ACC       // Uncomment to print acceleration to serial.
//#define PRINT_GYRO      // Uncomment to print gyroscope to serial.
//#define PRINT_MAG       // Uncomment to print magnetometer to serial.

#define PRINT_ORIENTATION   // Uncomment to print device orientation.
//#define PRINT_WORLD_ACC     // Uncomment to print acceleration transformed to world frame.
//#define PRINT_LINEAR_ACC    // Uncomment to print linear acceleration.

#define CALIBRATE_AMT 100     // The number of data to use to find offsets for sensor data.
#define LPF_ACC       1.0     // Percentage of new acc value to use.
#define LPF_GYRO      0.1     // Percentage of new gyro value to use.

/**
 * Construct StreamEx, for more print helper methods.
 */
StreamEx serial = Serial;

/**
 * Construct LSM object.
 */
LSM9DS1 lsm;

/**
 * Construct the accelerometer/gyroscope/magnetometer calibrators.
 */
Vector3Calibrator accCalibrator, gyroCalibrator, magCalibrator;

/**
 * Construct the OrientationEstimator, used to calculate orientation of device.
 */
OrientationEstimator orientationEstimator;

/**
 * Construct the AccelerometerDataTransformer, used to transform acc to world frame.
 */
AccelerometerDataTransformer accDataTransformer;

/**
 * Setup. Called once when Arduino starts up.
 */
void setup() {

  // Start serial
  Serial.begin(115200);

  if (!lsm.begin()) {
    Serial.println("Cannot find LSMDS1! Check your wiring.");
    while(1);
  }

  // Show starting message.
  Serial.println("Inertial tracking starting... DO NOT MOVE THE DEVICE!");
  delay(1000);
  Serial.println("Setting up everything...");
  
  // Initialize lsm
  lsm.begin();
  // init accelerometer
  lsm.setAccelScale(A_SCALE_2G);
  //lsm.setAccelODR(XL_ODR_119);
  // init gyroscope
  lsm.setGyroScale(G_SCALE_245DPS);
  //lsm.setGyroScale(G_SCALE_2000DPS);
  //lsm.setGyroODR(G_ODR_149);
  // init magnetometer
  lsm.setMagScale(M_SCALE_4GS);
  //lsm.setMagODR(M_ODR_125);

  // Calibration loop
  for (int i = 0; i < 50+CALIBRATE_AMT; i++) {
    // Read data from sensor.
    lsm.readAccel();
    lsm.readGyro();
    lsm.readMag();
    
    // Skip the first 50 data to stabalize sensor data.
    if (i < 50) {
      continue;
    }

    // Calibrate accelerometer
    Vector3 acc(lsm.calcAccel(lsm.ax), lsm.calcAccel(lsm.ay), lsm.calcAccel(lsm.az));
    accCalibrator.calibrate(acc);

    // Calibrate gyroscope
    Vector3 gyro(lsm.calcGyro(lsm.gx), lsm.calcGyro(lsm.gy), lsm.calcGyro(lsm.gz));
    gyroCalibrator.calibrate(gyro);

    // Calibrate magnetometer
    Vector3 mag(lsm.calcMag(-lsm.mx), lsm.calcMag(lsm.my), lsm.calcMag(lsm.mz));
    magCalibrator.calibrate(mag);

    delay(10); // TODO set to ODR??
  }

  // Show end setting up message.
  Serial.println("Done setting up. You may now move the device!");
  
  // Print plotter headers
  Serial.println("X,Y,Z");
}

/**
 * Loop function. Called as fast as possibile.
 */
void loop() {
  static Vector3 prevAcc;   // the acceleration from previous reading
  static Vector3 prevGyro;  // the gyroscope data from previous reading
  static Vector3 prevMag;   // the magnetometer data from previous reading
  
  // Get acceleration
  lsm.readAccel();
  //Vector3 acc(lsm.ax, lsm.ay, lsm.az);
  Vector3 acc(lsm.calcAccel(lsm.ax), lsm.calcAccel(lsm.ay), lsm.calcAccel(lsm.az));
  // Apply low pass filter
  //accLpf.filtering3(&acc);
  /*float accLpfRatio = 0.9;
  acc.x = accLpfRatio*prevAcc.x + (1-accLpfRatio)*acc.x;
  acc.y = accLpfRatio*prevAcc.y + (1-accLpfRatio)*acc.y;
  acc.z = accLpfRatio*prevAcc.z + (1-accLpfRatio)*acc.z;*/
  #ifdef PRINT_ACC
    serial.printf("%.6f,%.6f,%.6f\n", acc.x, acc.y, acc.z);
  #endif

  
  // Get gyroscope
  lsm.readGyro();
  //Vector3 gyro(lsm.gx, lsm.gy, lsm.gz);
  Vector3 gyro(lsm.calcGyro(lsm.gx), lsm.calcGyro(lsm.gy), lsm.calcGyro(lsm.gz));
  gyroCalibrator.apply(&gyro); // apply calibration
  // Apply low pass filter.
  gyro.x = LPF_GYRO*gyro.x + (1-LPF_GYRO)*prevGyro.x;
  gyro.y = LPF_GYRO*gyro.y + (1-LPF_GYRO)*prevGyro.y;
  gyro.z = LPF_GYRO*gyro.z + (1-LPF_GYRO)*prevGyro.z;
  // Clone gyro data to unpGyro, which does not include threshold and deg adjustment.
  // This will be used by prevGyro.
  Vector3 unpGyro(gyro.x, gyro.y, gyro.z);
  
  // Apply threshold to values.
  if (abs(gyro.x) <= 0.35) {
    gyro.x = 0.0;
  }
  if (abs(gyro.y) <= 0.13) {
    gyro.y = 0.0;
  }
  if (abs(gyro.z) <= 0.1) {
    gyro.z = 0.0;
  }
  // Scale gyro values to degrees.
  gyro.x *= 1.18;
  gyro.y *= 1.18;
  gyro.z *= 1.18;

  // Find the threshold - keep device stationary.
  /*static double maxX,maxY,maxZ;
  bool changed = false;
  if (abs(gyro.x) > maxX) {
    maxX = abs(gyro.x);
    changed = true;
  }
  if (abs(gyro.y) > maxY) {
    maxY = abs(gyro.y);
    changed = true;
  }
  if (abs(gyro.z) > maxZ) {
    maxZ = abs(gyro.z);
    changed = true;
  }
  if (changed) {
    serial.printf("%.6f, %.6f, %.6f\n", maxX, maxY, maxZ);
  }*/
  
  #ifdef PRINT_GYRO
    serial.printf("%.6f,%.6f,%.6f\n", gyro.x, gyro.y, gyro.z);
  #endif
  orientationEstimator.onGyroscopeData(gyro);


  // Get magnetometer
  lsm.readMag();
  Vector3 mag(lsm.calcMag(-lsm.mx), lsm.calcMag(lsm.my), lsm.calcMag(lsm.mz));
  #ifdef PRINT_MAG
    serial.printf("%.6f,%.6f,%.6f\n", mag.x, mag.y, mag.z);
  #endif

  
  // Get device orientation.
  orientationEstimator.fuse();
  EulerAngles orientation = orientationEstimator.getOrientationInEulerAngles();
  #ifdef PRINT_ORIENTATION
    //Serial.print("Orientation:,");
    //Serial.println(orientation.toString());
    static unsigned long curTime = micros();
    //Serial.print(micros() - curTime);Serial.print(",");
    Serial.print(orientation.roll);Serial.print(",");Serial.print(orientation.pitch);Serial.print(",");Serial.println(orientation.yaw);
  #endif
  // Send new orientation to accDataTransformer
  accDataTransformer.updateOrientation(orientation);

  // Adjusted acceleration (will be used for rotating coord frame and subtracting gravity)
  Vector3 adjAcc(acc.x, acc.y, acc.z);
  
  // Rotate acceleration from sensor-frame to world-frame
  accDataTransformer.rotateToWorldFrame(&adjAcc);
  #ifdef PRINT_WORLD_ACC
    //serial.printf("%f,%f,%f\n", acc.x, acc.y, acc.z);
    Serial.print(adjAcc.x);Serial.print(",");Serial.print(adjAcc.y);Serial.print(",");Serial.println(adjAcc.z);
  #endif

  //filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z);
  /*double roll = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
  double pitch = asin(2*(q0*q2 - q3*q1));
  double yaw = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));*/
  //Serial.print(filter.getRoll());Serial.print(",");Serial.print(filter.getPitch());Serial.print(",");Serial.println(filter.getYaw());
  
/*
  

  // Get linear acceleration (remove gravity from acc)
  Vector3 linearAcc(acc.x, acc.y, acc.z);
  accDataTransformer.subtractGravity(&linearAcc);
  #ifdef PRINT_LINEAR_ACC
    //serial.printf("%f,%f,%f\n", linearAcc.x, linearAcc.y, linearAcc.z);
    Serial.print(linearAcc.x);Serial.print(",");Serial.print(linearAcc.y);Serial.print(",");Serial.println(linearAcc.z);
  #endif*/


  /********* TEMP *************/
  /*
  static int calibCounter = 0;
  static Vector3Calibrator linearAccCalibrator;
  if (calibCounter < 200) {
    calibCounter++;
    if (calibCounter > 50)
      linearAccCalibrator.calibrate(linearAcc);
  } else {
    linearAccCalibrator.apply(&linearAcc);
    //Serial.print(linearAcc.x);Serial.print(",");Serial.print(linearAcc.y);Serial.print(",");Serial.println(linearAcc.z);

    unsigned long timeCurrent = micros(); // get current time in us
    static unsigned long timeLast = 0;
    // Calculate the time b/n this and the last reading
    if (timeLast != 0) {
      int32_t dt_us = timeCurrent - timeLast;
      double dt_s = dt_us / 1000000.0;

      static Vector3 vel;
      static Vector3 pos;
      static Vector3 prevLinearAcc;

      Vector3 prevVel(vel.x, vel.y, vel.z);
      vel.x += ((linearAcc.x+prevLinearAcc.x)/2.0) * dt_s;
      vel.y += ((linearAcc.y+prevLinearAcc.y)/2.0) * dt_s;
      vel.z += ((linearAcc.z+prevLinearAcc.z)/2.0) * dt_s;
      //Serial.println(vel.toString());

      pos.x += ((vel.x+prevVel.x)/2.0) * dt_s;
      pos.y += ((vel.y+prevVel.y)/2.0) * dt_s;
      pos.z += ((vel.z+prevVel.z)/2.0) * dt_s;
      Serial.println(pos.z);

      prevLinearAcc.x = linearAcc.x; prevLinearAcc.y = linearAcc.y; prevLinearAcc.z = linearAcc.z;
    }
    timeLast = timeCurrent;
  }
  /************** END TEMP ******************/

  // Update previous.
  prevAcc.x = acc.x; prevAcc.y = acc.y; prevAcc.z = acc.z;
  prevGyro.x = unpGyro.x; prevGyro.y = unpGyro.y; prevGyro.z = unpGyro.z;
  prevMag.x = mag.x; prevMag.y = mag.y; prevMag.z = mag.z;
}
