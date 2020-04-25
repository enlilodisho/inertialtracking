/*
 * InertialTracking
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Calculates orientation and relative position using a LSM9DS1 sensor.
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
#include <thread>
#include "../include/LSM9DS1-RPI/LSM9DS1.hpp"
#include "../include/INS.hpp"

// Num samples to use for Gyro calibration
#define G_CALIBRATION_AMT  100

// Sensitivity constants
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

template<class VECTOR3_T>
struct Vector3 {
    VECTOR3_T x;
    VECTOR3_T y;
    VECTOR3_T z;
};
static Vector3<double> gyroCalibration;
//static struct SensorData gyroCalibration;

static LSM9DS1 lsm;
static INS ins;

void loop();

/**
 * Main function.
 */
int main(int argc, char * argv[]) {
    std::cout << std::endl;
    std::cout << "Inertial Tracking v0.1" << std::endl;

    /* Set up LSM9SD1 */
    std::cout << "Setting up LSM9DS1...\n";
    // Set ODR rate for acc & gyro
    lsm.set_ag_odr(AG_ODR::AG_ODR_238); // settings faster than 238Hz are too fast for rpi
    // Set acc scale
    lsm.set_a_scale(A_SCALE::A_SCALE_4);
    // Set gyro scale
    lsm.set_g_scale(G_SCALE::G_SCALE_500);
    // Enable the data-ready status registers
    //lsm.set_drdy_enable_bit(true);
    // Turn on FIFO buffer
    lsm.set_fifo_enable_bit(true); // Enable FIFO feature
    lsm.set_fifo_mode(FIFO_MODE_BYPASS); // Reset FIFO buffer contents
    //lsm.set_fifo_mode(FIFO_MODE_ON); // Use regular FIFO mode. WARNING: If buffer gets full, output will stop until reset!
    lsm.set_fifo_mode(FIFO_MODE_ON_CONTINUOUS);
    // Throw away 1500 samples.
    int numSamplesRead = 0;
    while (numSamplesRead < 1500) {
        BYTE fifoStatus = lsm.get_fifo_status();
        uint8_t numUnreadInFIFO = lsm.get_num_fifo_unread(fifoStatus);
        if (numUnreadInFIFO > 0) {
            lsm.get_angular_rate();
            lsm.get_linear_acc();
            numSamplesRead++;
        }
    }

    // Calibrate gyro. (Find constant offset.)
    std::cout << "DO NOT MOVE DEVICE! Calibrating gyro...\n";
    gyroCalibration.x = 0; gyroCalibration.y = 0; gyroCalibration.z = 0;
    numSamplesRead = 0;
    while (numSamplesRead < G_CALIBRATION_AMT) {
        BYTE fifoStatus = lsm.get_fifo_status();
        uint8_t numUnreadInFIFO = lsm.get_num_fifo_unread(fifoStatus);
        if (numUnreadInFIFO > 0) {
            struct SensorData gyro = lsm.get_angular_rate();
            lsm.get_linear_acc();
            gyroCalibration.x += gyro.x;
            gyroCalibration.y += gyro.y;
            gyroCalibration.z += gyro.z;

            numSamplesRead++;
        }
    }
    gyroCalibration.x /= G_CALIBRATION_AMT;
    gyroCalibration.y /= G_CALIBRATION_AMT;
    gyroCalibration.z /= G_CALIBRATION_AMT;
    printf("Gyro calibration result: %f,%f,%f\n",
            gyroCalibration.x, gyroCalibration.y, gyroCalibration.z);

    std::cout << "DONE SETTING UP EVERYTHING!\n";

    // Start the INS
    ins.start();

    // start read data loop on new thread 
    std::thread imu_data_update_thread (loop);
    imu_data_update_thread.join(); // pause until thread finishes executing

    // Power down sensor
    std::cout << "Powering down LSM9DS1...\n";
    lsm.set_ag_odr(AG_ODR::AG_ODR_0);

    std::cout << std::endl;
}

void loop() {
    static int i = 0;

    while (i++ < 40000) {

        BYTE fifoStatus = lsm.get_fifo_status();
        uint8_t numUnreadInFIFO = lsm.get_num_fifo_unread(fifoStatus);
        if (numUnreadInFIFO > 0) {
            struct SensorData gyro  = lsm.get_angular_rate(); // ALWAYS READ GYRO DATA FIRST
            struct SensorData acc   = lsm.get_linear_acc();   // ALWAYS READ ACC  DATA SECOND

            // Apply calibration offsets
            gyro.x -= gyroCalibration.x;
            gyro.y -= gyroCalibration.y;
            gyro.z -= gyroCalibration.z;

            //printf("acc:%d,%d,%d ; gyro:%d,%d,%d\n", acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);

            // Apply sensitivity constants
            double gyroX = gyro.x, gyroY = gyro.y, gyroZ = gyro.z;
            gyroX *= SENSITIVITY_GYROSCOPE_500;
            gyroY *= SENSITIVITY_GYROSCOPE_500;
            gyroZ *= SENSITIVITY_GYROSCOPE_500;
            double accX = acc.x, accY = acc.y, accZ = acc.z;
            accX *= SENSITIVITY_ACCELEROMETER_4;
            accY *= SENSITIVITY_ACCELEROMETER_4;
            accZ *= SENSITIVITY_ACCELEROMETER_4;

            ins.onGyroscopeData(gyroX, gyroY, gyroZ, ((1/238.0)*1000000000));
            ins.onAccelerometerData(accX, accY, accZ, ((1/238.0)*1000000000));
        } else {
            //printf("Awaiting new data from sensor...\n");
        }

    }
}
