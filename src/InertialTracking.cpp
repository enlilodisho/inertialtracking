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
#include "../include/LSM9DS1-RPI/LSM9DS1.hpp"

LSM9DS1 lsm;

/**
 * Main function.
 */
int main(int argc, char * argv[]) {
    std::cout << "Hello world!" << std::endl;
    //lsm.set_ag_odr(AG_ODR::AG_ODR_952); // too fast for raspberry pi
    lsm.set_ag_odr(AG_ODR::AG_ODR_476);
    lsm.set_g_scale(G_SCALE::G_SCALE_2000);
    lsm.set_drdy_enable_bit(true);
    //std::cout << "Status: " << lsm.get_status_reg() << std::endl;
    int i = 0;
    while (i < 10000) {
        unsigned char status = lsm.get_status_reg();
        /*std::cout << "ACC: " << ((lsm.is_acc_available(status) == true) ? "true" : "false") << std::endl;
        std::cout << "GYRO: " << ((lsm.is_gyro_available(status) == true) ? "true" : "false") << std::endl;
        std::cout << "TEMP: " << ((lsm.is_temp_available(status) == true) ? "true" : "false") << std::endl;
        std::cout << std::endl;*/

        bool accAvail = lsm.is_acc_available(status);
        if (accAvail == true) {
            //std::cout << "ACC: true" << std::endl;
            //std::cout << "getLinearAcc" << std::endl;
            struct SensorData acc = lsm.get_linear_acc();
            printf("acc: %d, %d, %d\n", acc.x, acc.y, acc.z);
            //std::cout << " done\n";
        } else {
            std::cout << "waiting new data...\n";
        }

        i++;
    }
}
