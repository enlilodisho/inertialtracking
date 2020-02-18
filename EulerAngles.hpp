#ifndef EULER_ANGLES_HPP
#define EULER_ANGLES_HPP

#include <Arduino.h> // used only for Strings

struct EulerAngles
{
  double roll;
  double pitch;
  double yaw;  

  EulerAngles(); // constructor
  EulerAngles(double roll, double pitch, double yaw); // constructor
  EulerAngles toDegrees();
  EulerAngles toRadians();
  String toString();
  String toDegreesString();
  String toRadiansString();
};

#endif /* EULER_ANGLES_HPP */
