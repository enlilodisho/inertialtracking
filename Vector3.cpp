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

/**
 * Computes dot product of 2 Vector3's.
 */
double Vector3::Dot(Vector3 vecA, Vector3 vecB) {
  return vecA.x*vecB.x + vecA.y*vecB.y + vecA.z*vecB.z;
}

/**
 * Computes cross product of 2 Vector3's.
 */
Vector3 Vector3::Cross(Vector3 vecA, Vector3 vecB) {
  Vector3 cross;
  cross.x = vecA.y*vecB.z - vecA.z*vecB.y;
  cross.y = vecA.z*vecB.x - vecA.x*vecB.z;
  cross.z = vecA.x*vecB.y - vecA.y*vecB.x;
  return cross;
}

/**
 * Computes square magnitude of vector.
 */
double Vector3::SqrMagnitude(Vector3 vec) {
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}
