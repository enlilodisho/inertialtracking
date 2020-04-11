#ifndef QUATERNION_HPP
#define QUATERNION_HPP

struct EulerAngles {
    double pitch;
    double roll;
    double yaw;
};

class Quaternion {
    public:
        double w, x, y, z;

        // Constructors
        Quaternion();
        Quaternion(double w, double x, double y, double z);

        // Multiplication
        Quaternion operator*(const Quaternion& q2);
        Quaternion& operator*=(const Quaternion& q2);
        // Division
        Quaternion operator/(double k);
        Quaternion& operator/=(double k);
        Quaternion operator/(const Quaternion& q2);
        Quaternion& operator/=(const Quaternion& q2);

        // Operations
        Quaternion conjugate();
        double norm_squared();
        Quaternion inverse();

        // Conversions
        EulerAngles toEulerAngles();
};

#endif /* QUATERNION_HPP */
