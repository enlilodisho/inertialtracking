#include "Vector3.hpp"
#include <math.h>

/**
 * Constructor for Vector3.
 */
Vector3::Vector3(): x(0), y(0), z(0) {}

/**
 * Constructor for Vector3 with initial values.
 */
Vector3::Vector3(double x, double y, double z): x(x), y(y), z(z) {}

/**
 * Calculates and returns the magnitude of the vector.
 */
double Vector3::getMagnitude() {
  return sqrt(x*x + y*y + z*z);
}

/**
 * Converts vector in string format: "x,y,z".
 */
String Vector3::toString() {
  return String(x,6) + "," + String(y,6) + "," + String(z,6);
}
