#pragma once

#include <iostream>
#include <cmath>
#include <cassert>
#include "Vec3.h"
#include "Math_Util.h"

namespace CrunchMath {

	struct Quaternion
	{
		float w, x, y, z;

		Quaternion();

		Quaternion(float m_w, float m_x, float m_y, float m_z);

		Quaternion(float w, Vec3 v);

		Quaternion(const Quaternion& Q);

		void SetToIdentity();
		
		Quaternion& operator=(const Quaternion& Q);

		void SetToRotateAboutX(float radian);

		Quaternion& FromObjectToWorldEuler(Vec3&& hbp); 

		Quaternion& FromWorldToObjectEuler(Vec3&& hbp);

		void SetToRotateAboutY(float radian);

		void SetToRotateAboutZ(float radian);

		void SetToRotateAboutAxis(Vec3 v, float radian);

		Quaternion operator*(const Quaternion& Q);

		Quaternion& operator+=(const Quaternion& Q);

		Quaternion& operator*=(const Quaternion& Q);

		float GetRotationAngle();

		Vec3 GetRotationAxis();

		void Normalize();

	};

	float DotProduct(const Quaternion& Q, const Quaternion& P);
	
	Quaternion Expo(/*Parameter*/);

	Quaternion Slerp(/*Parameter*/);

	Quaternion Conjugate(const Quaternion& Q);
}