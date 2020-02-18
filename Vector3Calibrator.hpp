#ifndef VECTOR3_CALIBRATOR_HPP
#define VECTOR3_CALIBRATOR_HPP

#include "Vector3.hpp"
#include "EulerAngles.hpp"

class Vector3Calibrator {
  public:
    Vector3Calibrator();   // constructor
    ~Vector3Calibrator();  // destructor
    void calibrate(Vector3 data); // improve calibration
    void calibrate(EulerAngles anglesData); // improve calibration using EulerAngles
    void apply(Vector3 *data); // applies calibration
    void apply(EulerAngles *angles); // applies calibration to EulerAngles
  private:
    Vector3 calibration;
    int calibrationAmt;
};

#endif /* VECTOR3_CALIBRATOR_HPP */
