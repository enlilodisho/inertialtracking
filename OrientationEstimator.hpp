#ifndef ORIENTATION_ESTIMATOR_HPP
#define ORIENTATION_ESTIMATOR_HPP

#include "Vector3.hpp"
#include "EulerAngles.hpp"
#include "Quaternion.hpp"

class OrientationEstimator {
  public:
    OrientationEstimator();   // constructor
    ~OrientationEstimator();  // destructor
    void onAccelerometerData(Vector3 data);
    void onMagnetometerData(Vector3 data);
    void onGyroscopeData(Vector3 data);
    void fuse();  // Fuses orientation from different sources together.
    EulerAngles getOrientationInEulerAngles();
  private:
    EulerAngles orientation; // fused orientation.
    EulerAngles accMagEulerOrientation, gyroEulerOrientation;
    Quaternion quatOrientation;
};

#endif /* ORIENTATION_ESTIMATOR_HPP */
