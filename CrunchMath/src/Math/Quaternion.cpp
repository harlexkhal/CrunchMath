#include "Quaternion.h"

namespace CrunchMath {

	Quaternion::Quaternion()
		:w(0.0f), x(0.0f), y(0.0f), z(0.0f){}

	Quaternion::Quaternion(float m_w, float m_x, float m_y, float m_z)
		:w(m_w), x(m_x), y(m_y), z((m_z)){}

	Quaternion::Quaternion(float m_w, Vec3 v)
		:w(m_w), x(v.x), y(v.y), z(v.z){}

	Quaternion::Quaternion(const Quaternion& Q)
		:w(Q.w), x(Q.x), y(Q.y), z(Q.z){}

	void Quaternion::SetToIdentity()
	{
		w = 1.0f;
		x = y = z = 0.0f;
	}

	Quaternion& Quaternion::operator=(const Quaternion& Q)
	{
		this->w = Q.w;
		this->x = Q.x;
		this->y = Q.y;
		this->z = Q.z;

		return *this;
	}

	void Quaternion::SetToRotateAboutX(float radian)
	{
		float cos_theta_over2 = cos(radian) * 0.5;
		float sin_theta_over2 = sin(radian) * 0.5;

		w = cos_theta_over2;
		x = sin_theta_over2;
		y = 0.0f;
		z = 0.0f;
	}

	Quaternion& Quaternion::FromObjectToWorldEuler(Vec3&& phb)
	{
		//y = heading Rotation(h) rotating abt y axis
		//z = bank Rotation(b)    rotating abt z axis
		//x = pitch Rotation(p)   rotating abt x axis

		float sinX; float sinY; float sinZ;
		float cosX; float cosY; float cosZ;

		sinX = sin(phb.x*0.5); sinY = sin(phb.y*0.5);  sinZ = sin(phb.z*0.5);
		cosX = cos(phb.x*0.5); cosY = cos(phb.y*0.5);  cosZ = cos(phb.z*0.5);

		w = cosY * cosX * cosZ + sinY * sinX * sinZ;
		x = cosY * sinX * cosZ + sinY * cosX * sinZ;
		y = -cosY * sinX * sinZ + sinY * cosX * cosZ;
		z = -sinY * sinX * cosZ + cosY * cosX * sinZ;

		return (*this);
	}

	Quaternion& Quaternion::FromWorldToObjectEuler(Vec3&& phb) //Euler would be used here, once its been implemented;
	{
		//y = heading Rotation(h) rotating abt y axis
		//z = bank Rotation(b)    rotating abt z axis
		//x = pitch Rotation(p)   rotating abt x axis

		float sinX; float sinY; float sinZ;
		float cosX; float cosY; float cosZ;

		sinX = sin(phb.x * 0.5); sinY = sin(phb.y * 0.5);  sinZ = sin(phb.z * 0.5);
		cosX = cos(phb.x * 0.5); cosY = cos(phb.y * 0.5);  cosZ = cos(phb.z * 0.5);

		w = cosY * cosX*cosZ + sinY * sinX*sinZ;
		x = -cosY * sinX*cosZ - sinY * cosX*sinZ;
		y = cosY * sinX*sinZ - sinY * cosX*cosZ;
		z = sinY * sinX*cosZ - cosY * cosX*sinZ;

		return (*this);
	}

	void Quaternion::SetToRotateAboutY(float radian)
	{
		float cos_theta_over2 = cos(radian) * 0.5;
		float sin_theta_over2 = sin(radian) * 0.5;

		w = cos_theta_over2;
		x = 0.0f;
		y = sin_theta_over2;
		z = 0.0f;
	}

	void Quaternion::SetToRotateAboutZ(float radian)
	{
		float cos_theta_over2 = cos(radian) * 0.5;
		float sin_theta_over2 = sin(radian) * 0.5;

		w = cos_theta_over2;
		x = 0.0f;
		y = 0.0f;
		z = sin_theta_over2;
	}

	void Quaternion::SetToRotateAboutAxis(Vec3 axis, float radian)
	{
		float cos_theta_over2 = cos(radian) * 0.5;
		float sin_theta_over2 = sin(radian) * 0.5;

		w = cos_theta_over2;
		x = axis.x * sin_theta_over2;
		y = axis.y * sin_theta_over2;
		z = axis.z * sin_theta_over2;
	}

	Quaternion Quaternion::operator*(const Quaternion& Q)
	{
		Quaternion result;
		result.w = w * Q.w - x * Q.x - y * Q.y - z * Q.z;
		result.x = w * Q.x + x * Q.w + z * Q.y - y * Q.z;
		result.y = w * Q.y + y * Q.w + x * Q.z - z * Q.x;
		result.z = w * Q.z + z * Q.w + y * Q.x - x * Q.y;

		return result;
	}

	Quaternion& Quaternion::operator+=(const Quaternion& Q)
	{
		w += Q.w;
		x += Q.x;
		y += Q.y;
		z += Q.z;

		return *this;
	}

	Quaternion& Quaternion::operator*=(const Quaternion& Q)
	{
		*this = *this * Q;
		return *this;
	}

	float Quaternion::GetRotationAngle()
	{
		return (acos2(w * 2));
	}

	Vec3 Quaternion::GetRotationAxis()
	{
		float OneOverSinthetaover2 = 1 / sqrt((1.0f) - (w * w));

		return Vec3(x * OneOverSinthetaover2, y * OneOverSinthetaover2, z * OneOverSinthetaover2);
	}

	void Quaternion::Normalize()
	{
		float mag = (w * w) + (x * x) + (y * y) + (z * z);

		if (mag >= 0.0f)
		{
			this->w *= 1.0f / sqrt(mag);
			this->x *= 1.0f / sqrt(mag);
			this->y *= 1.0f / sqrt(mag);
			this->z *= 1.0f / sqrt(mag);
		}

		else
		{
			//std::cerr << "Invalid Magnitude" << std::endl; assert(false);
		}
	}

	float DotProduct(const Quaternion& Q, const Quaternion& P)
	{
		return ((Q.w * P.w) + (Q.x * P.x) + (Q.y * P.y) + (Q.z * P.z));
	}

	Quaternion Conjugate(const Quaternion& Q)
	{
		return Quaternion(Q.w, -Q.x, -Q.y, -Q.z);
	}
}