#ifndef VECTOR3_INTEGRATOR_HPP
#define VECTOR3_INTEGRATOR_HPP

#include "utils.hpp"

class Vector3Integrator {
    public:
        Vector3Integrator();   // constructor
        ~Vector3Integrator();  // destructor

        // Event handler for when new data to integrate.
        void onNewData(Vector3<double> newData, double dt_ns);

        // Getter(s)
        Vector3<double> getResult();
    private:
        Vector3<double> result;
        Vector3<double> prevData;
};

#endif /* VECTOR3_INTEGRATOR_HPP */
