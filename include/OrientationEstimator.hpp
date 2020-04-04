#ifndef ORIENTATION_ESTIMATOR_HPP
#define ORIENTATION_ESTIMATOR_HPP

#include "Quaternion.hpp"

class OrientationEstimator {
    public:
        OrientationEstimator();   // constructor
        ~OrientationEstimator();  // destructor

        // Event Handlers
        void onGyroscopeData(double pitch, double roll, double yaw, double dt_ns);
        void onMagnetometerData(double x, double y, double z, double dt_ns);

        // Getters
        Quaternion getOrientation();
    private:
        Quaternion orientation; // current orientation as a quaternion
};

#endif /* ORIENTATION_ESTIMATOR_HPP */
