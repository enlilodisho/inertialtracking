/*
 * Offset Calibrator
 * Copyright (c) 2020  Enlil Odisho
 * ------------------------------------------------
 * Applies offset to Vector3.
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
#include "Vector3Calibrator.hpp"

/**
 * Constructor for Vector3Calibrator.
 */
Vector3Calibrator::Vector3Calibrator() {}

/**
 * Destructor for Vector3Calibrator.
 */
Vector3Calibrator::~Vector3Calibrator() {}

/**
 * Improves calibration so that passed in data is closer to zero.
 */
void Vector3Calibrator::calibrate(Vector3 data) {
  calibration.x = (calibration.x * calibrationAmt + data.x) / (calibrationAmt+1);
  calibration.y = (calibration.y * calibrationAmt + data.y) / (calibrationAmt+1);
  calibration.z = (calibration.z * calibrationAmt + data.z) / (calibrationAmt+1);
  calibrationAmt++;
}
void Vector3Calibrator::calibrate(EulerAngles anglesData) {
  calibration.x = (calibration.x * calibrationAmt + anglesData.roll) / (calibrationAmt+1);
  calibration.y = (calibration.y * calibrationAmt + anglesData.pitch) / (calibrationAmt+1);
  calibration.z = (calibration.z * calibrationAmt + anglesData.yaw) / (calibrationAmt+1);
  calibrationAmt++;
}

/**
 * Applies calibration to data.
 */
void Vector3Calibrator::apply(Vector3 *data) {
  data->x -= calibration.x;
  data->y -= calibration.y;
  data->z -= calibration.z;
}
void Vector3Calibrator::apply(EulerAngles *angles) {
  angles->roll -= calibration.x;
  angles->pitch -= calibration.y;
  angles->yaw -= calibration.z;
}
