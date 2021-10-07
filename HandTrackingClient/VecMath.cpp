#include "HandTrackingClient/VecMath.h"
#include <iostream>

namespace HandTrackingClient
{

template <typename T>
Vector3<T>
Vector3<T>::cross (const Vector3<T>& rhs) const
{
    const Vector3<T>& a = *this;
    const Vector3<T>& b = rhs;
    return Vector3<T> (a.y*b.z - a.z*b.y,
                       a.z*b.x - a.x*b.z,
                       a.x*b.y - a.y*b.x);
}

template <typename T>
void
Vector3<T>::normalize()
{
    const T len = norm();
    if (len == T(0))
        return;

    x /= len;
    y /= len;
    z /= len;
}

template <typename T>
std::ostream& 
operator<< (std::ostream& os, const Vector3<T>& rhs)
{
    os << rhs.x << " " << rhs.y << " " << rhs.z;
    return os;
}

template <typename T>
Quaternion<T>
Quaternion<T>::conjugate() const
{
    return Quaternion<T> (-v, w);
}

template <typename T>
void
Quaternion<T>::normalize()
{
    const T sqrLen = v.squaredNorm() + w*w;
    const T len = std::sqrt(sqrLen);
    if (len == T(0))
        return;

    v /= len;
    w /= len;
}

template <typename T>
Vector3<T>
Quaternion<T>::rotate (const Vector3<T>& v_in) const
{
    // Rotation of a vector by q is equivalent to conjugation by q,
    //    q * v * q^{-1}  (where q^{-1} is the same as q's conjugate for unit quaternions).
    // Note that this is certainly not the most efficient way to implement this:
    return ((*this) * Quaternion<T>(v_in, 0.0f) * (*this).conjugate()).v;
}

template <typename T>
Quaternion<T> 
Quaternion<T>::fromAxisAngle (const Vector3<T> axis, const T angle)
{
    const T halfAngle = angle / T(2);
    return Quaternion<T> (axis.normalized() * static_cast<T>(sin(halfAngle)), cos(halfAngle));
}

template <typename T>
std::ostream& 
operator<< (std::ostream& os, const Quaternion<T>& rhs)
{
    os << rhs.v << " " << rhs.w;
    return os;
}

template <typename T>
void 
Transform<T>::invert()
{
    const Quaternion<T> invRot = rotation.conjugate();
    rotation = invRot;
    translation = -(invRot.rotate(translation));
}

template <typename T>
Matrix4<T>
Matrix4<T>::identity()
{
    Matrix4<T> result;
    result.data[0] = 1;
    result.data[5] = 1;
    result.data[10] = 1;
    result.data[15] = 1;
    return result;
}

template <typename T>
Matrix4<T>::Matrix4(Quaternion<T> q, Vector3<T> t)
{
    // Convert a Quaternion into a rotation matrix
    // http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_.E2.86.94_quaternion
    data[0] = 1 - 2*q.v.y*q.v.y - 2*q.v.z*q.v.z;
    data[1] = 2*q.v.x*q.v.y + 2*q.v.z*q.w;
    data[2] = 2*q.v.x*q.v.z - 2*q.v.y*q.w;
    data[3] = 0;
    data[4] = 2*q.v.x*q.v.y - 2*q.v.z*q.w;
    data[5] = 1 - 2*q.v.x*q.v.x - 2*q.v.z*q.v.z;
    data[6] = 2*q.v.y*q.v.z + 2*q.v.x*q.w;
    data[7] = 0;
    data[8] = 2*q.v.x*q.v.z + 2*q.v.y*q.w;
    data[9] = 2*q.v.y*q.v.z - 2*q.v.x*q.w;
    data[10] = 1 - 2*q.v.x*q.v.x - 2*q.v.y*q.v.y;
    data[11] = 0;
    data[12] = t.x;
    data[13] = t.y;
    data[14] = t.z;
    data[15] = 1;
}

template class Vector3<float>;
template class Vector3<double>;

template std::ostream& operator<< (std::ostream& os, const Vector3<float>& rhs);
template std::ostream& operator<< (std::ostream& os, const Vector3<double>& rhs);

template class Quaternion<float>;
template class Quaternion<double>;

template std::ostream& operator<< (std::ostream& os, const Quaternion<float>& rhs);
template std::ostream& operator<< (std::ostream& os, const Quaternion<double>& rhs);

template class Transform<float>;
template class Transform<double>;

template class Matrix4<float>;
template class Matrix4<double>;

} // namespace HandTrackingClient

