#ifndef ACCELEROMETER_DATA_TRANSFORMER_HPP
#define ACCELEROMETER_DATA_TRANSFORMER_HPP

#include "Vector3.hpp"
#include "EulerAngles.hpp"

class AccelerometerDataTransformer {
  public:
    AccelerometerDataTransformer();   // constructor
    ~AccelerometerDataTransformer();  // destructor
    void updateOrientation(EulerAngles orientation);
    void subtractGravity(Vector3 *data);
    void rotateToWorldFrame(Vector3 *data);
  private:
    Vector3 *gravity;
    Vector3 tGravity;
    EulerAngles orientation;
};

#endif /* ACCELEROMETER_DATA_TRANSFORMER_HPP */
