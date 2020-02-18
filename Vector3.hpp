#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include <Arduino.h> // used only for Strings

class Vector3
{
  public:
    double x;
    double y;
    double z;
  
    Vector3(); // constructor
    Vector3(double x, double y, double z); // constructor
    double getMagnitude();
    String toString();
};

#endif /* VECTOR3_HPP */
