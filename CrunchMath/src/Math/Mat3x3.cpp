#include "Mat3x3.h"

namespace CrunchMath
{
	Mat3x3::Mat3x3()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Matrix[i][j] = 0.0f;
			}
		}
	}

	Mat3x3::Mat3x3(const Vec3& col1, const Vec3& col2, const Vec3& col3)
	{
		//column vec 1               column vec 2            column vec 3
		Matrix[0][0] = col1.x;   Matrix[1][0] = col2.x;   Matrix[2][0] = col3.x;
		Matrix[0][1] = col1.y;   Matrix[1][1] = col2.y;   Matrix[2][1] = col3.y;
		Matrix[0][2] = col1.z;   Matrix[1][2] = col2.z;   Matrix[2][2] = col3.z;
	}

	Mat3x3::Mat3x3(const Vec3& col1, const Vec3& col2)
	{
		//column vec 1               column vec 2            column vec 3
		Matrix[0][0] = col1.x;   Matrix[1][0] = col2.x;   Matrix[2][0] = 0.0f;
		Matrix[0][1] = col1.y;   Matrix[1][1] = col2.y;   Matrix[2][1] = 0.0f;
		Matrix[0][2] = 0.0f;     Matrix[1][2] = 0.0f;     Matrix[2][2] = 1.0f;
	}

	Mat3x3::Mat3x3(const Mat3x3& m)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Matrix[i][j] = m.Matrix[i][j];
			}
		}
	}

	Mat3x3::Mat3x3(const Mat4x4& m)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Matrix[i][j] = m.Matrix[i][j];
			}
		}
	}

	Mat3x3::Mat3x3(float identity)
	{
		//column vec 1               column vec 2            column vec 3
		Matrix[0][0] = identity;   Matrix[1][0] = 0.0f;      Matrix[2][0] = 0.0f;
		Matrix[0][1] = 0.0f;       Matrix[1][1] = identity;  Matrix[2][1] = 0.0f;
		Matrix[0][2] = 0.0f;       Matrix[1][2] = 0.0f;      Matrix[2][2] = identity;
	}

	Mat3x3& Mat3x3::operator=(const Mat3x3& rhs)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Matrix[i][j] = rhs.Matrix[i][j];
			}
		}
		return *this;
	}

	Mat3x3 Mat3x3::Multiply(const Mat3x3& rhs)
	{
		Mat3x3 result;
		float hold = 0.0f;

		for (int h = 0; h < 3; h++)
		{
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					hold += (Matrix[j][h] * rhs.Matrix[i][j]);
				}

				result.Matrix[i][h] = hold;
				hold = 0.0f;
			}
		}
		return result;
	}

	Mat3x3& Mat3x3::operator*=(const Mat3x3& rhs)
	{
		*this = this->Multiply(rhs);
		return *this;
	}

	Mat3x3& Mat3x3::operator*=(float scalar)
	{
		//column vec 1            column vec 2             column vec 3
		Matrix[0][0] *= scalar;   Matrix[1][0] *= scalar;  Matrix[2][0] *= scalar;
		Matrix[0][1] *= scalar;   Matrix[1][1] *= scalar;  Matrix[2][1] *= scalar;
		Matrix[0][2] *= scalar;   Matrix[1][2] *= scalar;  Matrix[2][2] *= scalar;

		return *this;
	}

	Mat3x3& Mat3x3::operator+=(const Mat3x3& rhs)
	{
		//column vec 1            column vec 2             column vec 3
		Matrix[0][0] += Matrix[0][0];   Matrix[1][0] += Matrix[1][0];  Matrix[2][0] += Matrix[2][0];
		Matrix[0][1] += Matrix[0][1];   Matrix[1][1] += Matrix[1][1];  Matrix[2][1] += Matrix[2][1];
		Matrix[0][2] += Matrix[0][2];   Matrix[1][2] += Matrix[1][2];  Matrix[2][2] += Matrix[2][2];

		return *this;
	}

	void Mat3x3::InsertDiagonal(const Vec3& pos)
	{
		Matrix[0][0] = pos.x;
		Matrix[1][1] = pos.y;
		Matrix[2][2] = pos.z;
	}

	void Mat3x3::SetSkewSymmetric(const Vec3 vec)
	{
		//column vec 1            column vec 2            column vec 3
		Matrix[0][0] = 0.0f;      Matrix[1][0] = -vec.z;  Matrix[2][0] = vec.y;
		Matrix[0][1] = vec.z;     Matrix[1][1] = 0.0f;    Matrix[2][1] = -vec.x;
		Matrix[0][2] = -vec.y;    Matrix[1][2] = vec.x;   Matrix[2][2] = 0.0f;
	}

	Mat3x3& Mat3x3::Transpose()
	{
		*this = Mat3x3(

			Vec3(Matrix[0][0], Matrix[1][0], Matrix[2][0]), //Row Vector Arrangement; (vec1)

			Vec3(Matrix[0][1], Matrix[1][1], Matrix[2][1]),                        // (vec2)

			Vec3(Matrix[0][2], Matrix[1][2], Matrix[2][2])                         // (vec3)
		);

		return *this;
	}

	Vec3 Mat3x3::GetColumnVector(int i) const
	{
		return Vec3(Matrix[i][0], Matrix[i][1], Matrix[i][2]);
	}

	void Mat3x3::SetRotate(const Quaternion& q)
	{
		//column vec 1...
		Matrix[0][0] = 1.0f - (2.0f) * ((q.y * q.y) + (q.z * q.z));
		Matrix[0][1] = (2.0f) * ((q.x * q.y) + (q.w * q.z));
		Matrix[0][2] = (2.0f) * ((q.x * q.z) - (q.w * q.y));

		//column vec2...
		Matrix[1][0] = (2.0f) * ((q.x * q.y) - (q.w * q.z));
		Matrix[1][1] = 1.0f - (2.0f) * ((q.x * q.x) + (q.z * q.z));
		Matrix[1][2] = (2.0f) * ((q.y * q.z) + (q.w * q.x));

		//column vec3...
		Matrix[2][0] = (2.0f) * ((q.x * q.z) + (q.w * q.y));
		Matrix[2][1] = (2.0f) * ((q.y * q.z) - (q.w * q.x));
		Matrix[2][2] = 1.0f - (2.0f) * ((q.x * q.x) + (q.y * q.y));
	}

	Mat3x3 Invert(const Mat3x3& refm)
	{
		Mat3x3 m = refm;

		if (m.RotationMatrix)
		{
			m.Transpose();
			return m;
		}

		float det = Determinant(m);
		//assert(det == 0.0f);
		float oneoverdet = 1.0f / det;

		Mat3x3 Temp = m;
		//Column Vector 1
		m.Matrix[0][0] = ((Temp.Matrix[1][1] * Temp.Matrix[2][2]) - (Temp.Matrix[2][1] * Temp.Matrix[1][2])) * oneoverdet;
		m.Matrix[0][1] = -((Temp.Matrix[0][1] * Temp.Matrix[2][2]) - (Temp.Matrix[2][1] * Temp.Matrix[0][2])) * oneoverdet;
		m.Matrix[0][2] = ((Temp.Matrix[0][1] * Temp.Matrix[1][2]) - (Temp.Matrix[1][1] * Temp.Matrix[0][2])) * oneoverdet;
		//Column Vector 2
		m.Matrix[1][0] = -((Temp.Matrix[1][0] * Temp.Matrix[2][2]) - (Temp.Matrix[2][0] * Temp.Matrix[1][2])) * oneoverdet;
		m.Matrix[1][1] = ((Temp.Matrix[0][0] * Temp.Matrix[2][2]) - (Temp.Matrix[2][0] * Temp.Matrix[0][2])) * oneoverdet;
		m.Matrix[1][2] = -((Temp.Matrix[0][0] * Temp.Matrix[1][2]) - (Temp.Matrix[1][0] * Temp.Matrix[0][2])) * oneoverdet;
		//Column Vector 3
		m.Matrix[2][0] = ((Temp.Matrix[1][0] * Temp.Matrix[2][1]) - (Temp.Matrix[2][0] * Temp.Matrix[1][1])) * oneoverdet;
		m.Matrix[2][1] = -((Temp.Matrix[0][0] * Temp.Matrix[2][1]) - (Temp.Matrix[2][0] * Temp.Matrix[0][1])) * oneoverdet;
		m.Matrix[2][2] = ((Temp.Matrix[0][0] * Temp.Matrix[1][1]) - (Temp.Matrix[1][0] * Temp.Matrix[0][1])) * oneoverdet;

		return m;
	}
}