#pragma once
#include "Vec3.h"

namespace CrunchMath {

	struct Sphere
	{
		Vec3 CenterPosition;
		float Radius;

		Sphere();

		Sphere(Vec3 Center ,float Radius);

		Sphere(const Sphere& SphereShape);

		Sphere& operator=(const Sphere& SphereShape);

		bool NarrowPhaseCollisionTest(const Sphere& Object2);

		void Set(Vec3 Center, float Radius);
	};
}