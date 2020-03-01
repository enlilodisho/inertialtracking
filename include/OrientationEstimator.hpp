#ifndef ORIENTATION_ESTIMATOR_HPP
#define ORIENTATION_ESTIMATOR_HPP

#include "Quaternion.hpp"

class OrientationEstimator {
    public:
        OrientationEstimator();   // constructor
        ~OrientationEstimator();  // destructor

        // Event Handlers
        void onAccelerometerData(Vector3 data);
        void onMagnetometerData(Vector3 data);
        void onGyroscopeData(Vector3 data);

        // Getters
        Quaternion getOrientation();
    private:
        Quaternion orientation; // current orientation as a quaternion
};

#endif /* ORIENTATION_ESTIMATOR_HPP */
