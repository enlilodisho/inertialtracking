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
#include <chrono>
#include <set>
#include "../include/utils.hpp"
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

static Vector3<double> gyroCalibration;
static Vector3<double> magCalibration;
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
    // Set ODR rate for acc, gyro, and mag
    lsm.set_ag_odr(AG_ODR::AG_ODR_238); // settings faster than 238Hz are too fast for rpi
    lsm.set_m_odr(M_ODR::M_ODR_10);
    // Set acc scale
    lsm.set_a_scale(A_SCALE::A_SCALE_4);
    // Set gyro scale
    lsm.set_g_scale(G_SCALE::G_SCALE_500);
    // Set mag scale
    lsm.set_m_scale(M_SCALE::M_SCALE_4);
    // Turn on temperature compensation for mag data.
    lsm.set_m_temp_compensate(true);
    // Use highest available performance for mag data
    lsm.set_m_xy_performance(M_XY_PERFORMANCE_ULTRA);
    lsm.set_m_z_performance(M_Z_PERFORMANCE_ULTRA);
    // Enable the data-ready status registers
    //lsm.set_drdy_enable_bit(true);

    // Turn on magnetometer.
    lsm.set_m_mode(M_MODE_CONTINUOUS);

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

    // Calibrate mag.
    std::cout << "MOVE THE DEVICE AROUND ALL AXES! Calibrating Mag...\n";
    {
        // temp vars
        auto startTime = std::chrono::high_resolution_clock::now();
        std::set<int16_t> mag_x_data, mag_y_data, mag_z_data;
        int timeoutUs = 3000000; // 3 seconds

        struct SensorData magi = lsm.get_mag_field();
        Vector3<int16_t> minRead, maxRead = {x:magi.x, y:magi.y, z:magi.z};

        // Collect magenetometer data.
        while (timeoutUs > 0) {
            // Check if time to read.
            auto curTime = std::chrono::high_resolution_clock::now();
            long long elapsed = std::chrono::duration_cast
                <std::chrono::microseconds>(curTime - startTime).count();
            if (elapsed < ((1/10.0)*1000000)) {
                continue;
            }

            // Get mag reading.
            struct SensorData mag = lsm.get_mag_field();

            // Update min max
            bool mmUpdated = false;
            if (mag.x < minRead.x) { minRead.x = mag.x; mmUpdated = true; }
            else if (mag.x > maxRead.x) { maxRead.x = mag.x; mmUpdated = true; }
            if (mag.y < minRead.y) { minRead.y = mag.y; mmUpdated = true; }
            else if (mag.y > maxRead.y) { maxRead.y = mag.y; mmUpdated = true; }
            if (mag.z < minRead.z) { minRead.z = mag.z; mmUpdated = true; }
            else if (mag.z > maxRead.z) { maxRead.z = mag.z; mmUpdated = true; }
            if (mmUpdated) {
                timeoutUs = 3000000;
            } else {
                if ((maxRead.x - minRead.x) > 1000 &&
                    (maxRead.y - minRead.y) > 1000 &&
                    (maxRead.z - minRead.z) > 1000) {
                    timeoutUs -= elapsed;
                }
            }

            bool insertedNewData = false;
            // Insert data.
            auto x_it = mag_x_data.find(mag.x);
            if (x_it == mag_x_data.end()) {
                mag_x_data.insert(mag.x);
            } else insertedNewData = true;
            auto y_it = mag_y_data.find(mag.y);
            if (y_it == mag_y_data.end()) {
                mag_y_data.insert(mag.y);
            } else insertedNewData = true;
            auto z_it = mag_z_data.find(mag.z);
            if (z_it == mag_z_data.end()) {
                mag_z_data.insert(mag.z);
            } else insertedNewData = true;
            if (insertedNewData) {
                timeoutUs += elapsed;
            }

            startTime = curTime;
        }

        // Compute calibration for mag.
        // Get average of max and min
        magCalibration.x = (maxRead.x+minRead.x)/2.0;
        magCalibration.y = (maxRead.y+minRead.y)/2.0;
        magCalibration.z = (maxRead.z+minRead.z)/2.0;
        /* Mean
        std::set<int16_t>::iterator it;
        int n;
        magCalibration.x = 0;
        for (n = 0, it = mag_x_data.begin(); it != mag_x_data.end(); it++){
            magCalibration.x += *it;
            n++;
        }
        magCalibration.x /= n;
        magCalibration.y = 0;
        for (n = 0, it = mag_y_data.begin(); it != mag_y_data.end(); it++){
            magCalibration.y += *it;
            n++;
        }
        magCalibration.y /= n;
        magCalibration.z = 0;
        for (n = 0, it = mag_z_data.begin(); it != mag_z_data.end(); it++){
            magCalibration.z += *it;
            n++;
        }
        magCalibration.z /= n;
        */
        /* Median
        auto x_it = mag_x_data.begin();
        std::advance(x_it, mag_x_data.size() / 2);
        magCalibration.x = *x_it;
        auto y_it = mag_y_data.begin();
        std::advance(y_it, mag_y_data.size() / 2);
        magCalibration.y = *y_it;
        auto z_it = mag_z_data.begin();
        std::advance(z_it, mag_x_data.size() / 2);
        magCalibration.z = *z_it;
        */
        printf("Mag calibration result: %f,%f,%f\n",
                magCalibration.x, magCalibration.y, magCalibration.z);

        // Wait a couple seconds for user to stop moving device before
        // continuing.
        printf("Program paused for 3 seconds. Please stop moving device.\n");
        while (true) {
            // Check if time to read.
            auto curTime = std::chrono::high_resolution_clock::now();
            long long elapsed = std::chrono::duration_cast
                <std::chrono::seconds>(curTime - startTime).count();
            if (elapsed > 3) {
                break;
            }
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
    auto lastTime = std::chrono::high_resolution_clock::now();

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

        // Check if time to read mag data.
        auto curTime = std::chrono::high_resolution_clock::now();
        long long elapsed = std::chrono::duration_cast
            <std::chrono::microseconds>(curTime - lastTime).count();
        if (elapsed >= ((1/10.0)*1000000)) {
            struct SensorData mag = lsm.get_mag_field();

            mag.x -= magCalibration.x;
            mag.y -= magCalibration.y;
            mag.z -= magCalibration.z;

            ins.onMagnetometerData(mag.x, mag.y, mag.z, ((1/10.0)*1000000000));
            //printf("0,%d,%d,%d\n", mag.x, mag.y, mag.z);

            lastTime = curTime;
        }

    }
}
