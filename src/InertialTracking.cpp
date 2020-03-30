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
#include <chrono>
#include "../include/LSM9DS1-RPI/LSM9DS1.hpp"

LSM9DS1 lsm;

using namespace std::chrono;

/**
 * Main function.
 */
int main(int argc, char * argv[]) {
    std::cout << "Inertial Tracking starting..." << std::endl;

    lsm.set_ag_odr(AG_ODR::AG_ODR_119); // settings faster than 238Hz are too fast for rpi
    lsm.set_g_scale(G_SCALE::G_SCALE_2000);

    //lsm.set_drdy_enable_bit(true); // Enable data ready status registers
    
    // Turn on FIFO mode
    lsm.set_fifo_enable_bit(true); // Enable FIFO feature
    lsm.set_fifo_mode(FIFO_MODE_BYPASS); // reset FIFO buffer contents
    lsm.set_fifo_mode(FIFO_MODE_ON); // Using regular FIFO mode. If buffer gets full, output will stop until reset!

    static high_resolution_clock::time_point last = high_resolution_clock::now();
    high_resolution_clock::time_point now;

    int i = 0;
    while (i < 1000) {
        now = high_resolution_clock::now();

        unsigned char fifoStatus = lsm.get_fifo_status();
        uint8_t numFifoUnread = lsm.get_num_fifo_unread(fifoStatus);
        printf("[%d] Unread: %d, Overrun: %s, Threshold: %s", i,
                (int)numFifoUnread,
                ((lsm.did_fifo_overrun(fifoStatus) == true) ? "yes" : "no "),
                ((lsm.is_fifo_threshold_reached(fifoStatus) == true) ? "yes": "no "));

        if (numFifoUnread > 0) {
            duration<double, std::nano> dt = duration_cast<duration<double, std::nano>>(now - last);

            struct SensorData gyro = lsm.get_angular_rate();
            struct SensorData acc = lsm.get_linear_acc();
            printf(" ; acc:%d,%d,%d, gyro:%d,%d,%d ; [dt:%fus]", 
                    acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, dt.count());
        }

        std::cout << std::endl;

        i++;
    }
}
