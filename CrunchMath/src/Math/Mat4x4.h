#pragma once
#include <iostream>
#include <cmath>
#include <cassert>
#include "Vec3.h"
#include "Quaternion.h"

//***************************************************************************************
//MATRIX SETUP WITH COLUMN VECTORS ARRANGEMENT...Be WARNED...HARLEX TOLD YA...#C++ FUN!!!
//***************************************************************************************
//********************************
//********************************
//****************************************************************************
//Matrix Type Setting Feature to Enhance Optimization while inverting a matrix;
//Transpose Rotation matrix to form the Inverse M(Transpose)==M(Inverse)
//True For all Rotation Matrix...........No jokes...its how Matrix works.
//*****************************************************************************
 //*****************************************************************************
 //******************************************************************************
 //Every Mat4x4 Methods/Member Functions that begins with **SET** Resets the.....
 //current Matrix and slots in new Matrix data for all rows and columns depending
 //on what ***SET*** Methods you use. e.g matrix.SetExample() Reset the current 
 //Data of the Matrix and Resets it to what ever SetExample method those.........
 //Yeh!, thank me Later HarlexKhal Told you so.......peace out!!!..c you nxt comment.
 //********************************************************************************

namespace CrunchMath {

	struct Mat4x4
	{
		float Matrix[4][4];

		bool RotationMatrix = false;

		Mat4x4();

		Mat4x4(const Vec3& col1, const Vec3& col2, const Vec3& col3);

		Mat4x4(const Mat4x4& m);

		Mat4x4(float identiy);

		Mat4x4& operator=(const Mat4x4& rhs);

		void SetToIdentity();

        void Translate(const Vec3& pos);

		void ZeroTranslation();

		Mat4x4 Multiply(const Mat4x4& rhs);

		void InsertDiagonal(const float &value);

		void InsertDiagonal(const Vec3& pos);

		Mat4x4& RotateFromInertialToObject(const Quaternion& orient);
		
		Mat4x4& RotateFromObjectToInertial(const Quaternion& inverse_orient);

		void SetRotate(const int& axis, const float& radian);

		Mat4x4& Rotate(const Vec3& axis, const float& radian);

		Mat4x4& SetPerspective(float fov_radian, float aspect_ratio, float near, float far);

		Mat4x4& SetOrthographic(float fov_radian, float aspect_ratio, float near, float far);

		void SetToLookAt(Vec3& campos, Vec3& camtarget, Vec3& up);

		void SetRotate(const Vec3& axis, const float& radian);

		Mat4x4& Rotate(const Quaternion& q);

		void SetRotate(const Quaternion& q);

		void SetScale(const float& size);

		void SetScale(const Vec3& v);

		void Scale(const Vec3& v);

		void Scale(const float& s);

		Mat4x4& Transpose();

		Mat4x4& operator*=(const Mat4x4& rhs);

		Vec3 GetColumnVector(int i) const;
    };

	static inline Vec3 operator*(const Mat4x4& lhs, const Vec3& rhs)
	{
		return Vec3(  (lhs.Matrix[0][0] * rhs.x) + (lhs.Matrix[1][0] * rhs.y) + (lhs.Matrix[2][0] * rhs.z) + lhs.Matrix[3][0],
			          (lhs.Matrix[0][1] * rhs.x) + (lhs.Matrix[1][1] * rhs.y) + (lhs.Matrix[2][1] * rhs.z) + lhs.Matrix[3][1],
			          (lhs.Matrix[0][2] * rhs.x) + (lhs.Matrix[1][2] * rhs.y) + (lhs.Matrix[2][2] * rhs.z) + lhs.Matrix[3][2]
		           );
	}

	static inline Mat4x4 operator*( Mat4x4& lhs, const Mat4x4& rhs)
	{
			return lhs.Multiply(rhs);
    }

	static inline Vec3 GetTranslation(const Mat4x4& m)
	{
		return Vec3(m.Matrix[3][0], m.Matrix[3][1], m.Matrix[3][2]);
	}

	static inline float Determinant(const Mat4x4& m)
	{
		return((m.Matrix[0][0] * ((m.Matrix[1][1] * m.Matrix[2][2]) - (m.Matrix[2][1] * m.Matrix[1][2])))
			+ (-m.Matrix[1][0] * ((m.Matrix[0][1] * m.Matrix[2][2]) - (m.Matrix[2][1] * m.Matrix[0][2])))
			+ (m.Matrix[2][0] * ((m.Matrix[0][1] * m.Matrix[1][2]) - (m.Matrix[1][1] * m.Matrix[0][2])))
			);
	}

	Mat4x4 Invert(const Mat4x4& m);
}