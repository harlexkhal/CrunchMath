#include "Vec4.h"

namespace CrunchMath {

	Vec4::Vec4()
		:x(0.0f), y(0.0f), z(0.0f), a(0.0f) {}

	Vec4::Vec4(float xc, float yc, float zc, float ac)
		: x(xc), y(yc), z(zc), a(ac) {}

	Vec4::Vec4(const Vec4& u)
		: x(u.x), y(u.y), z(u.z) {}

	Vec4& Vec4::operator=(const Vec4& u)
	{
		this->x = u.x;
		this->y = u.y;
		this->z = u.z;
		this->a = u.a;

		return *this;
	}
}