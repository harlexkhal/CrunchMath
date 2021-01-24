#pragma once
#include <cassert>
#include <cmath>
#include <iostream>

namespace CrunchMath {

	struct Vec4
	{
		float x, y, z, a;
		
		Vec4();

		Vec4(float xc, float yc, float zc, float ac);

		Vec4(const Vec4& u);

		Vec4& operator=(const Vec4& u);
	};
}