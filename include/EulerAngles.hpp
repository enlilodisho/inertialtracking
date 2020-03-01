#ifndef EULER_ANGLES_HPP
#define EULER_ANGLES_HPP

#include <string>

class EulerAngles
{
    public:
        double roll;
        double pitch;
        double yaw;  

        // Constructors
        EulerAngles();
        EulerAngles(double roll, double pitch, double yaw);

        // Convert between degrees/radians
        EulerAngles toDegrees();
        EulerAngles toRadians();

        // Get string ROLL,PITCH,YAW
        std::string toString();
};

#endif /* EULER_ANGLES_HPP */
