#include "EulerAngles.hpp"

/**
 * Constructor for EulerAngles.
 */
EulerAngles::EulerAngles(): roll(0), pitch(0), yaw(0) {
  
}

/**
 * Constructor for EulerAngles with initial values.
 */
EulerAngles::EulerAngles(double roll, double pitch, double yaw): roll(roll), pitch(pitch), yaw(yaw) {}

/**
 * Converts EulerAngles to degrees.
 */
EulerAngles EulerAngles::toDegrees() {
  EulerAngles degAngles;
  degAngles.roll = roll * (180/PI);
  degAngles.pitch = pitch * (180/PI);
  degAngles.yaw = yaw * (180/PI);
  return degAngles;
}

/**
 * Converts EulerAngles to radians.
 */
EulerAngles EulerAngles::toRadians() {
  EulerAngles radAngles;
  radAngles.roll = roll * (PI/180.0);
  radAngles.pitch = pitch * (PI/180.0);
  radAngles.yaw = yaw * (PI/180.0);
  return radAngles;
}

/**
 * Converts EulerAngles in string format: "roll,pitch,yaw".
 */
String EulerAngles::toString() {
  return String(roll,6) + "," + String(pitch,6) + "," + String(yaw,6);
}

/**
 * Converts EulerAngles to degrees in string format: "roll,pitch,yaw".
 */
String EulerAngles::toDegreesString() {
  return String(roll*(180/PI),2) + "," + String(pitch*(180/PI),2) + "," + String(yaw*(180/PI),2);
}

/**
 * Converts EulerAngles to radians in string format: "roll,pitch,yaw".
 */
String EulerAngles::toRadiansString() {
  return String(roll*(PI/180.0),6) + "," + String(pitch*(PI/180.0),6) + "," + String(yaw*(PI/180.0),6);
}
