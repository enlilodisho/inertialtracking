#ifndef INERTIAL_NAVIGATION_SYSTEM_HPP
#define INERTIAL_NAVIGATION_SYSTEM_HPP

class INS {
    public:
        INS();   // constructor
        ~INS();  // destructor

        // Event Handlers
        void onAccelerometerData(double x, double y, double z);
        void onGyroscopeData(double pitch, double roll, double yaw);
        void onMagnetometerData(double x, double y, double z);
};

#endif /* INERTIAL_NAVIGATION_SYSTEM_HPP */
