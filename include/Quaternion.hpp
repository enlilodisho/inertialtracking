#ifndef QUATERNION_HPP
#define QUATERNION_HPP

class Quaternion {
    public:
        double w, x, y, z;

        // Constructors
        Quaternion();
        Quaternion(double w, double x, double y, double z);

        // Multiplication
        Quaternion operator*(const Quaternion& q2);
        Quaternion& operator*=(const Quaternion& q2);
};

#endif /* QUATERNION_HPP */
