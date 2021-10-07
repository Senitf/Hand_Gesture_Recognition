#pragma once

#include <algorithm>
#include <iosfwd>
#include <cstdint>
#include <cmath>

namespace HandTrackingClient
{

//! \brief A minimal 3-vector class.  
//!
//! We have provided this simple vector class to
//! avoid a dependence on an external math library
//! (like Eigen). 
template <typename T>
class Vector3
{
public:
    //! The x component.
    T x;

    //! The y component.
    T y;

    //! The z component.
    T z;

    //! The default constructor initializes to (0, 0, 0).
    Vector3<T>() : x(0), y(0), z(0) {}

    //! Initialize from the passed-in values.
    Vector3<T>(T x_in, T y_in, T z_in) : x(x_in), y(y_in), z(z_in) {}

    //! Convert to a vector with a different base type.
    template <typename T2>
    Vector3<T2> cast() const { return Vector3<T2> ((T2) x, (T2) y, (T2) z); }

    //! Adds the passed-in vector to this vector.
    Vector3<T>& operator+= (const Vector3<T>& rhs)     { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }

    //! Subtracts the passed-in vector to this vector.
    Vector3<T>& operator-= (const Vector3<T>& rhs)     { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }

    //! Unary minus operator.
    Vector3<T> operator-() const                       { return Vector3<T>(-x, -y, -z); }

    //! Multiplies this vector (in place) by the scalar.
    Vector3<T>& operator*= (const T rhs)               { x *= rhs; y *= rhs; z *= rhs; return *this; }

    //! Divides this vector (in place) by the scalar.
    Vector3<T>& operator/= (const T rhs)               { x /= rhs; y /= rhs; z /= rhs; return *this; }

    //! Computes the dot product of this vector with another.
    T dot (const Vector3<T>& rhs) const                { return x*rhs.x + y*rhs.y + z*rhs.z; }

    //! Computes the cross product of this vector with another.
    Vector3<T> cross (const Vector3<T>& rhs) const;

    //! Tests for equality (no floating point slop).
    bool operator== (const Vector3<T>& rhs) const      { return (x == rhs.x) && (y == rhs.y) && (z == rhs.z); }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Vector3<T>& rhs) const      { return !(*this == rhs); }

    //! Returns the L2 squared norm (x^2 + y^2 + z^2).
    T squaredNorm() const                              { return x*x + y*y + z*z; }

    //! Returns the l_infinity norm (the supremum over the absolute values of the elements).  
    T lInfNorm() const                                 { return std::max (std::max (std::abs(x), std::abs(y)), std::abs(z)); }

    //! Returns the L2 norm sqrt(x^2 + y^2 + z^2).
    T norm() const                                     { return std::sqrt(squaredNorm()); }

    //! Normalize this vector.  Note that it avoids the divide by zero
    //! in cases where the vector is equal to (0, 0, 0).
    void normalize();

    //! Returns a normalized version of this vector.
    Vector3<T> normalized() const                      { Vector3<T> result(*this); result.normalize(); return result; }
};

//! Multiplies a vector by a scalar.
template <typename T>
Vector3<T> operator* (const T lhs, const Vector3<T>& rhs)                  { Vector3<T> result(rhs); result *= lhs; return result; }

//! Multiplies a vector by a scalar.
template <typename T>
inline Vector3<T> operator* (const Vector3<T>& lhs, const T rhs)           { Vector3<T> result(lhs); result *= rhs; return result; }

//! Divides a vector by a scalar.
template <typename T>
inline Vector3<T> operator/ (const Vector3<T>& lhs, const T rhs)           { Vector3<T> result(lhs); result /= rhs; return result; }

//! Adds two vectors together.
template <typename T>
inline Vector3<T> operator+ (const Vector3<T>& lhs, const Vector3<T>& rhs) { Vector3<T> result(lhs); result += rhs; return result; }

//! Subtracts the second vector from the first.
template <typename T>
inline Vector3<T> operator- (const Vector3<T>& lhs, const Vector3<T>& rhs) { Vector3<T> result(lhs); result -= rhs; return result; }

//! Prints a vector to the output stream in the format "v.x v.y v.z".
template <typename T>
std::ostream& operator<< (std::ostream& os, const Vector3<T>& rhs);

typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

//! \brief A minimal quaternion class.  
//!
//! We have provided this simple Quaternion class to
//! avoid a dependence on an external math library
//! (like Eigen). 
template <typename T>
class Quaternion
{
public:
    //! The vector component
    Vector3<T> v;

    //! The w scalar component.
    T w;

    //! Initializes the quaternion to the identity rotation.
    Quaternion<T>() : v(0, 0, 0), w(1) {}

    //! Initializes the quaternion from the passed-in values.
    Quaternion<T>(T x_in, T y_in, T z_in, T w_in) 
        : v(x_in, y_in, z_in), w(w_in) {}

    //! \brief Initializes the quaternion from vector and scalar components.
    //! 
    //! Note that this is _not_ the same as initializing with an axis-angle
    //! pair (see \ref Quaternionf::fromAxisAngle).  
    //!
    //! @param v_in  Vector part.
    //! @param w_in  Scalar part.
    Quaternion<T>(const Vector3<T>& v_in, T w_in)
        : v(v_in), w(w_in) {}

    //! Convert to a vector with a different base type.
    template <typename T2>
    Quaternion<T2> cast() const { return Quaternion<T2> (v.template cast<T2>(), (T2) w); }

    //! Initializes the quaternion with the passed-in axis/angle pair.  
    //!
    //! @param axis  Rotation axis, must be normalized.
    //! @param angle Angle in radians.
    static Quaternion<T> fromAxisAngle (const Vector3<T> axis, const T angle);

    //! \brief Computes and returns the quaternion conjugate.  
    //! 
    //! For normalized quaternions (which represent rotations), the conjugate
    //! represents the inverse rotation.
    Quaternion<T> conjugate() const;

    //! Rotates the passed-in vector by the rotation represented by this quaternion.  
    Vector3<T> rotate (const Vector3<T>& v) const;

    //! Normalizes this quaternion.  
    void normalize();

    //! Returns a normalized version of this quaternion.   
    Quaternion<T> normalized() const { Quaternion<T> result (*this); result.normalize(); return result; }

    //! Tests for equality (no floating point slop).
    bool operator== (const Quaternion<T>& rhs) const    { return (v == rhs.v) && (w == rhs.w); }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Quaternion<T>& rhs) const    { return !(*this == rhs); }
};

//! Multiplies two quaternions.
template <typename T>
Quaternion<T> operator* (const Quaternion<T>& a, const Quaternion<T>& b)
{
    // From Wikipedia:
    // http://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
    return Quaternion<T> (a.w*b.v + b.w*a.v + a.v.cross(b.v),
                          a.w*b.w - a.v.dot(b.v));
}


//! Prints a quaternion to the output stream in the format "v.x v.y v.z w"
template <typename T>
std::ostream& operator<< (std::ostream& os, const Quaternion<T>& rhs);

typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;

/*! Represents the transform
    \f[
      \left[ \begin{array}{cc}
         \mathbf{R} & \mathbf{t} \\
         \mathbf{0} & 1
      \end{array} \right]
    \f]
    The final transformation is \f$ T(\mathbf{X}) = \mathbf{R}*\mathbf{x} + \mathbf{t} \f$, where \f$\mathbf{R}\f$ is the
    rotational component and \f$ \mathbf{t} \f$ is the translational component.
*/
template <typename T>
class Transform
{
public:
    //! This is the rotation component of the transform.  
    Quaternion<T> rotation;

    //! This is the translation component of the transform.
    Vector3<T> translation;

    //! Constructs a transform from the given rotation and translation.
    Transform (const Quaternion<T>& rotation_in, const Vector3<T>& translation_in)
        : rotation (rotation_in), translation (translation_in) {}

    Transform() {}

    //! Inverts this transform.
    void invert();

    //! Returns the inverse of this transform.
    Transform<T> inverse() const { Transform<T> result(*this); result.invert(); return result; }

    //! Convert to a transform with a different base type.
    template <typename T2>
    Transform<T2> cast() const { return Transform<T2> (rotation.template cast<T2>(), translation.template cast<T2>()); }

    //! Tests for equality (no floating point slop).
    bool operator== (const Transform<T>& rhs) const { return (rotation == rhs.rotation) && (translation == rhs.translation); }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Transform<T>& rhs) const { return !(*this == rhs); }
};

template <typename T>
Transform<T> operator* (const Transform<T>& lhs, const Transform<T>& rhs)
{
    // [ R_1 t_1 ] * [ R_2 t_2 ] = [ R_1*R_2  R_1*t_2 + t_1 ]
    // [  0   1  ]   [  0   1  ]   [    0           1       ]
    return Transform<T> (lhs.rotation * rhs.rotation, 
                         lhs.translation + lhs.rotation.rotate(rhs.translation));
}

template <typename T>
Vector3<T> operator* (const Transform<T>& lhs, const Vector3<T>& rhs)
{
    return lhs.translation + lhs.rotation.rotate (rhs);
}

typedef Transform<float> Transformf;
typedef Transform<double> Transformd;

//! 4x4 matrix stored in column major format.
//!
//! We have provided this simple Matrix4f class to
//! avoid a dependence on an external math library
//! (like Eigen).  
template <typename T>
class Matrix4
{
public:
    //! The 4x4 = 16 matrix entries, stored column-major.  
    T data[16];

    //! Initializes with the matrix of all 0s.
    Matrix4() { std::fill(data, data+16, T(0)); }

    static Matrix4 identity();

    //! Initializes with the upper-left 3x3 block set to the
    //! rotation specified by q and the translation component
    //! set to t.  
    Matrix4(Quaternion<T> q, Vector3<T> t);

    //! Column-wise element access.
    const T& operator() (unsigned iRow, unsigned jCol) const { return data[jCol * 4 + iRow]; }

    //! Column-wise element access.
    T& operator() (unsigned iRow, unsigned jCol)             { return data[jCol * 4 + iRow]; }

    //! Convert to a matrix with a different base type.
    template <typename T2>
    Matrix4<T2> cast() const
    {
        Matrix4<T2> result;
        for (int i = 0; i < 16; ++i)
            result.data[i] = (T2) data[i];
        return result;
    }

    //! Tests for equality (no floating point slop).
    bool operator== (const Matrix4<T>& rhs) const
    {
        for (int i = 0; i < 16; ++i)
        {
            if (data[i] != rhs.data[i])
                return false;
        }

        return true;
    }

    //! Tests for inequality (no floating point slop).
    bool operator!= (const Matrix4<T>& rhs) const { return !(*this == rhs); }
};

typedef Matrix4<float> Matrix4f;
typedef Matrix4<double> Matrix4d;

} // namespace HandTrackingClient

