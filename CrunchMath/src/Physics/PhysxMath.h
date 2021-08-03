#pragma once
#include <math.h>
#include "Core.h"

namespace CrunchPhysx {

    class Vector3
    {
    public:
        cpfloat x;
        cpfloat y;
        cpfloat z;

    public:
        Vector3() : x(0), y(0), z(0) {}
        Vector3(const cpfloat x, const cpfloat y, const cpfloat z)
            : x(x), y(y), z(z) {}

        cpfloat operator[](unsigned i) const
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        cpfloat& operator[](unsigned i)
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        void operator+=(const Vector3& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        Vector3 operator+(const Vector3& v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        void operator-=(const Vector3& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        Vector3 operator-(const Vector3& v) const
        {
            return Vector3(x-v.x, y-v.y, z-v.z);
        }

        void operator*=(const cpfloat value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        Vector3 operator*(const cpfloat value) const
        {
            return Vector3(x*value, y*value, z*value);
        }

        Vector3 componentProduct(const Vector3 &vector) const
        {
            return Vector3(x * vector.x, y * vector.y, z * vector.z);
        }

        Vector3 vectorProduct(const Vector3 &vector) const
        {
            return Vector3(y * vector.z - z * vector.y,
                           z * vector.x - x * vector.z,
                           x * vector.y - y * vector.x);
        }

        Vector3 operator%(const Vector3 &vector) const
        {
            return Vector3(y * vector.z - z * vector.y,
                           z * vector.x - x * vector.z,
                           x * vector.y - y * vector.x);
        }

        cpfloat scalarProduct(const Vector3 &vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        cpfloat operator *(const Vector3 &vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        void addScaledVector(const Vector3& vector, cpfloat scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        cpfloat magnitude() const
        {
            return cp_sqrt(x * x + y * y + z * z);
        }

        cpfloat squareMagnitude() const
        {
            return x * x + y * y + z * z;
        }

        void normalise()
        {
            cpfloat l = magnitude();
            if (l > 0)
            {
                (*this) *= ((cpfloat)1)/l;
            }
        }

        void clear()
        {
            x = y = z = 0;
        }
    };

    class Quaternion
    {
    public:
        union 
        {
            struct
            {
                cpfloat r;
                cpfloat i;
                cpfloat j;
                cpfloat k;
            };

            cpfloat data[4];
        };

        Quaternion() : r(1), i(0), j(0), k(0) {}

        Quaternion(const cpfloat r, const cpfloat i, const cpfloat j, const cpfloat k)
            : r(r), i(i), j(j), k(k)
        {}

        void normalise()
        {
            cpfloat d = r * r + i * i + j * j + k * k;
            
            if (d < cp_epsilon) 
            {
                r = 1;
                return;
            }

            d = ((cpfloat)1.0) / cp_sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }
        
        void operator *=(const Quaternion &multiplier)
        {
            Quaternion q = *this;
            r = q.r*multiplier.r - q.i*multiplier.i -
                q.j*multiplier.j - q.k*multiplier.k;
            i = q.r*multiplier.i + q.i*multiplier.r +
                q.j*multiplier.k - q.k*multiplier.j;
            j = q.r*multiplier.j + q.j*multiplier.r +
                q.k*multiplier.i - q.i*multiplier.k;
            k = q.r*multiplier.k + q.k*multiplier.r +
                q.i*multiplier.j - q.j*multiplier.i;
        }

        void addScaledVector(const Vector3& vector, cpfloat scale)
        {
            Quaternion q(0,
                vector.x * scale,
                vector.y * scale,
                vector.z * scale);
            q *= *this;
            r += q.r * ((cpfloat)0.5);
            i += q.i * ((cpfloat)0.5);
            j += q.j * ((cpfloat)0.5);
            k += q.k * ((cpfloat)0.5);
        }

        void rotateByVector(const Vector3& vector)
        {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
    };

    class Matrix4
    {
    public:
        cpfloat data[12];
        Matrix4()
        {
            data[1] = data[2] = data[3] = data[4] = data[6] =
                data[7] = data[8] = data[9] = data[11] = 0;
            data[0] = data[5] = data[10] = 1;
        }

        Matrix4 operator*(const Matrix4 &o) const
        {
            Matrix4 result;
            result.data[0] = (o.data[0]*data[0]) + (o.data[4]*data[1]) + (o.data[8]*data[2]);
            result.data[4] = (o.data[0]*data[4]) + (o.data[4]*data[5]) + (o.data[8]*data[6]);
            result.data[8] = (o.data[0]*data[8]) + (o.data[4]*data[9]) + (o.data[8]*data[10]);

            result.data[1] = (o.data[1]*data[0]) + (o.data[5]*data[1]) + (o.data[9]*data[2]);
            result.data[5] = (o.data[1]*data[4]) + (o.data[5]*data[5]) + (o.data[9]*data[6]);
            result.data[9] = (o.data[1]*data[8]) + (o.data[5]*data[9]) + (o.data[9]*data[10]);

            result.data[2] = (o.data[2]*data[0]) + (o.data[6]*data[1]) + (o.data[10]*data[2]);
            result.data[6] = (o.data[2]*data[4]) + (o.data[6]*data[5]) + (o.data[10]*data[6]);
            result.data[10] = (o.data[2]*data[8]) + (o.data[6]*data[9]) + (o.data[10]*data[10]);

            result.data[3] = (o.data[3]*data[0]) + (o.data[7]*data[1]) + (o.data[11]*data[2]) + data[3];
            result.data[7] = (o.data[3]*data[4]) + (o.data[7]*data[5]) + (o.data[11]*data[6]) + data[7];
            result.data[11] = (o.data[3]*data[8]) + (o.data[7]*data[9]) + (o.data[11]*data[10]) + data[11];

            return result;
        }

        Vector3 operator*(const Vector3 &vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[1] +
                vector.z * data[2] + data[3],

                vector.x * data[4] +
                vector.y * data[5] +
                vector.z * data[6] + data[7],

                vector.x * data[8] +
                vector.y * data[9] +
                vector.z * data[10] + data[11]
            );
        }

        Vector3 transform(const Vector3 &vector) const
        {
            return (*this) * vector;
        }

        cpfloat getDeterminant() const;
        void setInverse(const Matrix4 &m);

        Vector3 transformInverseDirection(const Vector3 &vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[4] +
                vector.z * data[8],

                vector.x * data[1] +
                vector.y * data[5] +
                vector.z * data[9],

                vector.x * data[2] +
                vector.y * data[6] +
                vector.z * data[10]
            );
        }

        Vector3 transformInverse(const Vector3 &vector) const
        {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];

            return Vector3(
                tmp.x * data[0] +
                tmp.y * data[4] +
                tmp.z * data[8],

                tmp.x * data[1] +
                tmp.y * data[5] +
                tmp.z * data[9],

                tmp.x * data[2] +
                tmp.y * data[6] +
                tmp.z * data[10]
            );
        }

        Vector3 getAxisVector(int i) const
        {
            return Vector3(data[i], data[i+4], data[i+8]);
        }

        void setOrientationAndPos(const Quaternion &q, const Vector3 &pos)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = pos.x;

            data[4] = 2*q.i*q.j - 2*q.k*q.r;
            data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[6] = 2*q.j*q.k + 2*q.i*q.r;
            data[7] = pos.y;

            data[8] = 2*q.i*q.k + 2*q.j*q.r;
            data[9] = 2*q.j*q.k - 2*q.i*q.r;
            data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
            data[11] = pos.z;
        }
    };

    class Matrix3
    {
    public:
        cpfloat data[9];

        Matrix3()
        {
            data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
                data[6] = data[7] = data[8] = 0;
        }

        Matrix3(const Vector3 &compOne, const Vector3 &compTwo,
            const Vector3 &compThree)
        {
            setComponents(compOne, compTwo, compThree);
        }

        Matrix3(cpfloat c0, cpfloat c1, cpfloat c2, cpfloat c3, cpfloat c4, cpfloat c5,
                                                    cpfloat c6, cpfloat c7, cpfloat c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        void setDiagonal(cpfloat a, cpfloat b, cpfloat c)
        {
            setInertiaTensorCoeffs(a, b, c);
        }

        void setInertiaTensorCoeffs(cpfloat ix, cpfloat iy, cpfloat iz, 
                                   cpfloat ixy=0, cpfloat ixz=0, cpfloat iyz=0)
        {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }

        void setBlockInertiaTensor(const Vector3 &halfSizes, cpfloat mass)
        {
            Vector3 squares = halfSizes.componentProduct(halfSizes);
            setInertiaTensorCoeffs(0.3f*mass*(squares.y + squares.z),
                0.3f*mass*(squares.x + squares.z),
                0.3f*mass*(squares.x + squares.y));
        }

        /**
         * Sets the matrix to be a skew symmetric matrix based on
         * the given vector. The skew symmetric matrix is the equivalent
         * of the vector product. So if a,b are vectors. a x b = A_s b
         * where A_s is the skew symmetric form of a.
         */
        void setSkewSymmetric(const Vector3 vector)
        {
            data[0] = data[4] = data[8] = 0;
            data[1] = -vector.z;
            data[2] = vector.y;
            data[3] = vector.z;
            data[5] = -vector.x;
            data[6] = -vector.y;
            data[7] = vector.x;
        }

        void setComponents(const Vector3 &compOne, const Vector3 &compTwo,
            const Vector3 &compThree)
        {
            data[0] = compOne.x;
            data[1] = compTwo.x;
            data[2] = compThree.x;
            data[3] = compOne.y;
            data[4] = compTwo.y;
            data[5] = compThree.y;
            data[6] = compOne.z;
            data[7] = compTwo.z;
            data[8] = compThree.z;

        }

        Vector3 operator*(const Vector3 &vector) const
        {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
            );
        }

        Vector3 transform(const Vector3 &vector) const
        {
            return (*this) * vector;
        }

        Vector3 transformTranspose(const Vector3 &vector) const
        {
            return Vector3
            (
                vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
                vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
                vector.x * data[2] + vector.y * data[5] + vector.z * data[8]
            );
        }

        Vector3 getAxisVector(int i) const
        {
            return Vector3(data[i], data[i+3], data[i+6]);
        }

        void setInverse(const Matrix3 &m)
        {
            cpfloat t4 = m.data[0] * m.data[4];
            cpfloat t6 = m.data[0] * m.data[5];
            cpfloat t8 = m.data[1] * m.data[3];
            cpfloat t10 = m.data[2] * m.data[3];
            cpfloat t12 = m.data[1] * m.data[6];
            cpfloat t14 = m.data[2] * m.data[6];

            // Calculate the determinant
            cpfloat t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
                           t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

            // Make sure the determinant is non-zero.
            if (t16 == (cpfloat)0.0f) return;
            cpfloat t17 = 1/t16;

            data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*t17;
            data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*t17;
            data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*t17;
            data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*t17;
            data[4] = (m.data[0]*m.data[8]-t14)*t17;
            data[5] = -(t6-t10)*t17;
            data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*t17;
            data[7] = -(m.data[0]*m.data[7]-t12)*t17;
            data[8] = (t4-t8)*t17;
        }

        Matrix3 inverse() const
        {
            Matrix3 result;
            result.setInverse(*this);
            return result;
        }

        void invert()
        {
            setInverse(*this);
        }

        void setTranspose(const Matrix3 &m)
        {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        Matrix3 transpose() const
        {
            Matrix3 result;
            result.setTranspose(*this);
            return result;
        }

        Matrix3 operator*(const Matrix3 &o) const
        {
            return Matrix3
            (
                    data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
                    data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
                    data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

                    data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
                    data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
                    data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

                    data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
                    data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
                    data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
            );
        }

        void operator*=(const Matrix3 &o)
        {
            cpfloat t1;
            cpfloat t2;
            cpfloat t3;

            t1 = data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6];
            t2 = data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7];
            t3 = data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8];
            data[0] = t1;
            data[1] = t2;
            data[2] = t3;

            t1 = data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6];
            t2 = data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7];
            t3 = data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8];
            data[3] = t1;
            data[4] = t2;
            data[5] = t3;

            t1 = data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6];
            t2 = data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7];
            t3 = data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8];
            data[6] = t1;
            data[7] = t2;
            data[8] = t3;
        }

        void operator*=(const cpfloat scalar)
        {
            data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
            data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
            data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
        }

        void operator+=(const Matrix3 &o)
        {
            data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
            data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
            data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
        }

        void setOrientation(const Quaternion &q)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = 2*q.i*q.j - 2*q.k*q.r;
            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[5] = 2*q.j*q.k + 2*q.i*q.r;
            data[6] = 2*q.i*q.k + 2*q.j*q.r;
            data[7] = 2*q.j*q.k - 2*q.i*q.r;
            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        }
    };
}