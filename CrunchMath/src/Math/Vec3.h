#pragma once
#include <cassert>
#include <cmath>
#include <iostream>

namespace CrunchMath {

	struct Vec3
	{
		float x, y, z;
		
		Vec3();

		Vec3(float xc, float yc, float zc);

		Vec3(const Vec3& u);

		float operator[](unsigned i) const;

		float& operator[](unsigned i);

		Vec3& operator=(const Vec3 &u);

		Vec3 operator+(const Vec3& u) const;

		Vec3& operator+=(const Vec3 &u);

		Vec3 operator-(const Vec3& u) const;

		Vec3& operator-=(const Vec3 &u);

		Vec3 operator*(const Vec3& u) const;

		Vec3& operator*=(const Vec3& u);

		Vec3 operator*(const float& Scale) const;

		Vec3& operator*=(const float& Scale);

		Vec3 operator/(const float& Scale) const;

		Vec3& operator/=(const float& Scale);

		Vec3 operator- () const;

		bool operator==(Vec3& u);

		bool operator!=(Vec3& u);

		void Normalize();
	};

	static inline Vec3 CrossProduct(const Vec3& lhs, const Vec3& rhs)
	{
		return Vec3
		(
			lhs.y * rhs.z - lhs.z * rhs.y,
			lhs.z * rhs.x - lhs.x * rhs.z,
			lhs.x * rhs.y - lhs.y * rhs.x
		);
	}

	static inline Vec3 Abs(const Vec3& v)
	{
		return Vec3(fabs(v.x), fabs(v.y), fabs(v.z));
	}

	static inline Vec3 Scale(const float Scaler, const Vec3& rhs)
	{
		return Vec3(Scaler * rhs.x, Scaler * rhs.y, Scaler * rhs.z);
	}

	static inline float Distance(const Vec3& lhs, const Vec3& rhs)
	{
		return
			(sqrt(
				((lhs.x - rhs.x) * (lhs.x - rhs.x)) +
				((lhs.y - rhs.y) * (lhs.y - rhs.y)) +
				((lhs.z - rhs.z) * (lhs.z - rhs.z))
			)
				);
	}

	static inline float DotProduct(const Vec3& lhs, const Vec3& rhs)
	{
		return ((lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z));
	}
}