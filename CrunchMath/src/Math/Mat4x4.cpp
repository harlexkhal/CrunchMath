#include "Mat4x4.h"

namespace CrunchMath {

	Mat4x4::Mat4x4()
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (i == 3 && j == 3)
				{
					Matrix[i][j] = 1;
					continue;
				}
				Matrix[i][j] = 0;
			}
		}
	}

	Mat4x4::Mat4x4(float identity)
	{
		//column vec 1               column vec 2            column vec 3                column  vec 4
		Matrix[0][0] = identity;   Matrix[1][0] = 0.0f;      Matrix[2][0] = 0.0f;      Matrix[3][0] = 0.0f;
		Matrix[0][1] = 0.0f;       Matrix[1][1] = identity;  Matrix[2][1] = 0.0f;      Matrix[3][1] = 0.0f;
		Matrix[0][2] = 0.0f;       Matrix[1][2] = 0.0f;      Matrix[2][2] = identity;  Matrix[3][2] = 0.0f;
		Matrix[0][3] = 0.0f;       Matrix[1][3] = 0.0f;      Matrix[2][3] = 0.0f;      Matrix[3][3] = 1.0f;
	}

	Mat4x4::Mat4x4(const Vec3& col1, const Vec3& col2, const Vec3& col3)
	{
		//column vec 1               column vec 2            column vec 3            column  vec 4
		Matrix[0][0] = col1.x;   Matrix[1][0] = col2.x;   Matrix[2][0] = col3.x;  Matrix[3][0] = 0.0f;
		Matrix[0][1] = col1.y;   Matrix[1][1] = col2.y;   Matrix[2][1] = col3.y;  Matrix[3][1] = 0.0f;
		Matrix[0][2] = col1.z;   Matrix[1][2] = col2.z;   Matrix[2][2] = col3.z;  Matrix[3][2] = 0.0f;
		Matrix[0][3] = 0.0f;     Matrix[1][3] = 0.0f;     Matrix[2][3] = 0.0f;    Matrix[3][3] = 1.0f;
	}

	Mat4x4::Mat4x4(const Mat4x4& m)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				Matrix[i][j] = m.Matrix[i][j];
			}
		}
	}

	Mat4x4& Mat4x4::operator=(const Mat4x4& rhs)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				Matrix[i][j] = rhs.Matrix[i][j];
			}
		}
		return *this;
	}

	void Mat4x4::SetToIdentity()
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (i == j)
				{
					Matrix[i][j] = 1;
					continue;
				}

				Matrix[i][j] = 0;
			}
		}
	}

	void Mat4x4::Translate(const Vec3& pos)
	{
		Matrix[3][0] = pos.x;
		Matrix[3][1] = pos.y;
		Matrix[3][2] = pos.z;
	}

	void Mat4x4::ZeroTranslation()
	{
		Matrix[3][0] = Matrix[3][1] = Matrix[3][2] = 0;
	}

	Mat4x4 Mat4x4::Multiply(const Mat4x4& rhs)
	{
		Mat4x4 result;
		float hold = 0.0f;

		for (int h = 0; h < 4; h++)
		{
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					hold += (Matrix[j][h] * rhs.Matrix[i][j]);
				}
				
				result.Matrix[i][h] = hold;
				hold = 0.0f;
			}
		}
		return result;
	}

	void Mat4x4::InsertDiagonal(const float &value)
	{
		Matrix[0][0] = Matrix[1][1] = Matrix[2][2] = value;
	}

	void Mat4x4::InsertDiagonal(const Vec3& pos)
	{
		Matrix[0][0] = pos.x;
		Matrix[1][1] = pos.y;
		Matrix[2][2] = pos.z;
	}

	Mat4x4& Mat4x4::RotateFromInertialToObject(const Quaternion& orient)
	{
		this->Rotate(orient);
		this->ZeroTranslation();
		return *this;
	}

	//This is very very very extremely slow, there are better ways to do this like just invert the converted quaternion and insert it invertedly directly...did this just To Test some Stuffs XD
    Mat4x4& Mat4x4::RotateFromObjectToInertial(const Quaternion& inverse_orient)
	{
		this->SetRotate(inverse_orient);         //I Should have called Rotate directly, but did this to set the matrix to a fresh Matrix. with no Translations
		this->RotationMatrix = true;
		*this = Invert(*this);
		
		return *this;
	}

	//Not the best of Rotation Algorithm,  doing this for Fun...
	void Mat4x4::SetRotate(const int &axis, const float& radian)
	{
		//Making sure axis is either 1, 2, or 3;... x, y, or z in that order...
		assert(axis <= 3 && axis > 0);

		float s = sin(radian);
		float c = cos(radian);

		switch (axis)
		{
		case 1:
		{
			Matrix[0][0] = 1.0f;   Matrix[1][0] = 0.0f;   Matrix[2][0] = 0.0f;
			Matrix[0][1] = 0.0f;   Matrix[1][1] = c;      Matrix[2][1] = -s;
			Matrix[0][2] = 0.0f;   Matrix[1][2] = s;      Matrix[2][2] = c;

			break;
		}

		case 2:
		{
			Matrix[0][0] = c;      Matrix[1][0] = 0.0f;    Matrix[2][0] = s;
			Matrix[0][1] = 0.0f;   Matrix[1][1] = 1.0f;    Matrix[2][1] = 0.0f;
			Matrix[0][2] = -s;     Matrix[1][2] = 0.0f;    Matrix[2][2] = c;

			break;
		}

		case 3:
		{
			Matrix[0][0] = c;     Matrix[1][0] = -s;       Matrix[2][0] = 0.0f;
			Matrix[0][1] = s;     Matrix[1][1] = c;        Matrix[2][1] = 0.0f;
			Matrix[0][2] = 0.0f;  Matrix[1][2] = 0.0f;     Matrix[2][2] = 1.0f;

			break;
		}

		default:
			assert(false);
			break;
		}

		this->ZeroTranslation();
	}

	Mat4x4& Mat4x4::Rotate(const Vec3& axis, const float& radian)
	{
		//Making sure its a unit Vector...
		assert(axis.x<=1.0f && axis.x>= 0.0f);
		assert(axis.y <= 1.0f && axis.y>= 0.0f);
		assert(axis.z <= 1.0f && axis.z >= 0.0f);

		float c = cos(radian);
		float s = sin(radian);

		float a = 1 - c;

		//column vec 1...
		Matrix[0][0] = ((axis.x*axis.x)*a) + c;
		Matrix[0][1] = ((axis.x*axis.y)*a) + axis.z*s;
		Matrix[0][2] = ((axis.x*axis.z)*a) - axis.y*s;

		//column vec2...
		Matrix[1][0] = ((axis.x*axis.y)*a) - axis.z*s;
		Matrix[1][1] = ((axis.y*axis.y)*a) + c;
		Matrix[1][2] = ((axis.y*axis.z)*a) + axis.x*s;

		//column vec3...
		Matrix[2][0] = ((axis.x*axis.z)*a) + axis.y*s;
		Matrix[2][1] = ((axis.y*axis.z)*a) - axis.x*s;
		Matrix[2][2] = ((axis.z*axis.z)*a) + c;

		return *this;
	}
	
	Mat4x4& Mat4x4::SetPerspective(float fov_radian, float aspect_ratio, float near, float far)
	{
		this->SetToIdentity();
		float zoom = 1 / tan(fov_radian * 0.5);
		float Phy_Sizeof_Window = zoom / aspect_ratio;

		this->Matrix[0][0] = Phy_Sizeof_Window;
		this->Matrix[1][1] = zoom;
		this->Matrix[2][2] = -(far + near) / (far - near);   //Negating cos of Opengl clip space left handed convention***(right handed specification MatrixSet)****
		this->Matrix[3][2] = -(2 * far * near) / (far - near); 
		this->Matrix[2][3] = -1.0f;       //same here   positive z points inward, negating it to point outwards.
		this->Matrix[3][3] = 0.0f;

		return (*this);
	}

	Mat4x4& Mat4x4::SetOrthographic(float fov_radian, float aspect_ratio, float near, float far)
	{
		this->SetToIdentity();
		float zoom = 1 / tan(fov_radian * 0.5);
		float Phy_Sizeof_Window = zoom / aspect_ratio;
		this->SetToIdentity();
	
		this->Matrix[0][0] = Phy_Sizeof_Window;
		this->Matrix[1][1] = zoom;
		this->Matrix[2][2] = -(2.0f) / (far - near);  //Negating cos of Opengl clip space left handed convention***(right handed specification MatrixSet)****
		this->Matrix[3][2] = -((far + near) / (far - near));

		return (*this);
	}

	void Mat4x4::SetToLookAt(Vec3& campos, Vec3& object, Vec3& up)
	{
		this->SetToIdentity();
		Vec3 CameraDirection;
		Vec3 CameraUp;
		Vec3 CameraRight;

		CameraDirection = campos - object;
		CameraDirection.Normalize();
		CameraRight = CrossProduct(up, CameraDirection);
		CameraRight.Normalize();
		CameraUp = CrossProduct(CameraDirection, CameraRight);
		CameraUp.Normalize();

		//Transposing The Matrix Straight Away..... Inverting the Camera Matrix. so the Scene Gets The movement. not the camera.
		Matrix[0][0] = CameraRight.x;       Matrix[1][0] = CameraRight.y;         Matrix[2][0] = CameraRight.z;
		Matrix[0][1] = CameraUp.x;          Matrix[1][1] = CameraUp.y;            Matrix[2][1] = CameraUp.z; 
		Matrix[0][2] = CameraDirection.x;   Matrix[1][2] = CameraDirection.y;     Matrix[2][2] = CameraDirection.z;

		Mat4x4 PosMatrix(1.0);
		PosMatrix.Translate(-campos);

		*this *= (PosMatrix);
	}

	void Mat4x4::SetRotate(const Vec3& axis, const float& radian)
	{
		this->Rotate(axis, radian);
		this->ZeroTranslation();
	}

	Mat4x4& Mat4x4::Rotate(const Quaternion& q)
	{
		//column vec 1...
		Matrix[0][0] = 1.0f - (2.0f) * ((q.y*q.y) + (q.z*q.z));
		Matrix[0][1] = (2.0f) * ((q.x*q.y) + (q.w*q.z));
		Matrix[0][2] = (2.0f) * ((q.x*q.z) - (q.w*q.y));

		//column vec2...
		Matrix[1][0] = (2.0f) * ((q.x*q.y) - (q.w*q.z));
		Matrix[1][1] = 1.0f - (2.0f) * ((q.x*q.x) + (q.z*q.z));
		Matrix[1][2] = (2.0f) * ((q.y*q.z) + (q.w*q.x));

		//column vec3...
		Matrix[2][0] = (2.0f) * ((q.x*q.z) + (q.w*q.y));
		Matrix[2][1] = (2.0f) * ((q.y*q.z) - (q.w*q.x));
		Matrix[2][2] = 1.0f - (2.0f) * ((q.x*q.x) + (q.y*q.y));

		return *this;
	}

	void Mat4x4::SetRotate(const Quaternion& q)
	{
		this->Rotate(q);
		this->ZeroTranslation();
	}

	void Mat4x4::SetScale(const float& size)
	{
		this->SetToIdentity();
		this->InsertDiagonal(size);
	}

	void Mat4x4::SetScale(const Vec3& pos)
	{
		this->SetToIdentity();
		this->InsertDiagonal(pos);
	}

	void Mat4x4::Scale(const Vec3& pos)
	{
		Mat4x4 Scale;
		Scale.InsertDiagonal(pos);
		*this *= Scale;
	}

	void Mat4x4::Scale(const float& s)
	{
		Mat4x4 Scale;
		Scale.InsertDiagonal(s);
		*this *= Scale;
	}

	Mat4x4& Mat4x4::Transpose()
	{	
		*this = Mat4x4(

			Vec3(Matrix[0][0], Matrix[1][0], Matrix[2][0]), //Row Vector Arrangement; (vec1)

			Vec3(Matrix[0][1], Matrix[1][1], Matrix[2][1]),                        // (vec2)

			Vec3(Matrix[0][2], Matrix[1][2], Matrix[2][2])                         // (vec3)
		);

		return *this;
	}

	Mat4x4& Mat4x4::operator*=(const Mat4x4& rhs)
	{
		*this = this -> Multiply(rhs);
		return *this;
	}
	
	Vec3 Mat4x4::GetColumnVector(int i) const
	{
		return Vec3(Matrix[i][0], Matrix[i][1], Matrix[i][2]);
	}

	Mat4x4 Invert(const Mat4x4& refm)
	{
		Mat4x4 m = refm;

		if (m.RotationMatrix)
		{
            Vec3 Trans = GetTranslation(m);
			m.Transpose();

			float x = -((m.Matrix[0][0] * Trans.x) + (m.Matrix[1][0] * Trans.y) + (m.Matrix[2][0] * Trans.z));
			float y = -((m.Matrix[0][1] * Trans.x) + (m.Matrix[1][1] * Trans.y) + (m.Matrix[2][1] * Trans.z));
			float z = -((m.Matrix[0][2] * Trans.x) + (m.Matrix[1][2] * Trans.y) + (m.Matrix[2][2] * Trans.z));
			m.Translate(Vec3(x, y, z));

			return m;
		}

		float det = Determinant(m);
		float oneoverdet = 1.0f / det;

		   Mat4x4 Temp = m;
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
		                                              //Translation Portion
		   float x = -((m.Matrix[0][0] * m.Matrix[3][0]) + (m.Matrix[1][0] * m.Matrix[3][1]) + (m.Matrix[2][0] * m.Matrix[3][2]));
		   float y = -((m.Matrix[0][1] * m.Matrix[3][0]) + (m.Matrix[1][1] * m.Matrix[3][1]) + (m.Matrix[2][1] * m.Matrix[3][2]));
		   float z = -((m.Matrix[0][2] * m.Matrix[3][0]) + (m.Matrix[1][2] * m.Matrix[3][1]) + (m.Matrix[2][2] * m.Matrix[3][2]));

		   m.Translate(Vec3(x, y, z));
		 
		return m;
	}
}