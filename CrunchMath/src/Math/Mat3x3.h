#pragma once
#include "Vec3.h"
#include "Mat4x4.h"
#include "Quaternion.h"

namespace CrunchMath{

	struct Mat3x3
	{
		float Matrix[3][3];

		bool RotationMatrix = false;

		Mat3x3();

		Mat3x3(const Vec3& col1, const Vec3& col2, const Vec3& col3);

		Mat3x3(const Vec3& col1, const Vec3& col2);

		Mat3x3(const Mat3x3& m);

		Mat3x3(const Mat4x4& m);

		Mat3x3(float identiy);

		Mat3x3& operator=(const Mat3x3& rhs);

		Mat3x3 Multiply(const Mat3x3& rhs);

		Mat3x3& operator*=(const Mat3x3& rhs);

		Mat3x3& operator*=(float scalar);

		Mat3x3& operator+=(const Mat3x3& rhs);

		void InsertDiagonal(const Vec3& pos);

		void SetSkewSymmetric(const Vec3 vec);

		Mat3x3& Transpose();

		Vec3 GetColumnVector(int i) const;

		void SetRotate(const Quaternion& q);
	};

	static inline Mat3x3 operator*(Mat3x3& lhs, const Mat3x3& rhs)
	{
		return lhs.Multiply(rhs);
	}

	static inline Vec3 operator*(const Mat3x3& lhs, const Vec3& rhs)
	{
		return Vec3(
			          (lhs.Matrix[0][0] * rhs.x) + (lhs.Matrix[1][0] * rhs.y) + (lhs.Matrix[2][0] * rhs.z),
			          (lhs.Matrix[0][1] * rhs.x) + (lhs.Matrix[1][1] * rhs.y) + (lhs.Matrix[2][1] * rhs.z),
			          (lhs.Matrix[0][2] * rhs.x) + (lhs.Matrix[1][2] * rhs.y) + (lhs.Matrix[2][2] * rhs.z)
		           );
	}

	static inline float Determinant(const Mat3x3& m)
	{
		return((m.Matrix[0][0] * ((m.Matrix[1][1] * m.Matrix[2][2]) - (m.Matrix[2][1] * m.Matrix[1][2])))
			+ (-m.Matrix[1][0] * ((m.Matrix[0][1] * m.Matrix[2][2]) - (m.Matrix[2][1] * m.Matrix[0][2])))
			+ (m.Matrix[2][0] * ((m.Matrix[0][1] * m.Matrix[1][2]) - (m.Matrix[1][1] * m.Matrix[0][2])))
			);
	}

	static inline Mat3x3 Abs(const Mat3x3& m)
	{
		Mat3x3 mMatrix;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				mMatrix.Matrix[i][j] = fabs(m.Matrix[i][j]);
			}
		}

		return mMatrix;
	}

	Mat3x3 Invert(const Mat3x3& m);
};