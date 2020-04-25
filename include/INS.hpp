#ifndef INERTIAL_NAVIGATION_SYSTEM_HPP
#define INERTIAL_NAVIGATION_SYSTEM_HPP

#include <deque>
#include <thread>
#include <mutex>
#include "OrientationEstimator.hpp"
#include "utils.hpp"
#include "Vector3Integrator.hpp"

class INS {
    public:
        INS();   // constructor
        ~INS();  // destructor

        void start();
        void loop();

        // Event Handlers
        void onGyroscopeData(double pitch, double roll, double yaw, double dt_ns);
        void onAccelerometerData(double x, double y, double z, double dt_ns);
        void onMagnetometerData(double x, double y, double z, double dt_ns);
    private:
        bool active;
        std::mutex queueMtx;

        struct QueueNode {
            double x;
            double y;
            double z;
            double dt_ns;
        };
        std::deque<struct QueueNode*> gyroQueue;
        std::deque<struct QueueNode*> accQueue;

        OrientationEstimator orientationEstimator;

        int lacc_calib_counter;
        Vector3<double> lacc_calibration;
        Vector3Integrator laccIntegrator;
        Vector3Integrator velIntegrator;
};

#endif /* INERTIAL_NAVIGATION_SYSTEM_HPP */
