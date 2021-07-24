#include "Sphere.h"

namespace CrunchMath {

	Sphere::Sphere()
	{
		Radius = 0.0f;
		CenterPosition = Vec3(0.0f, 0.0f, 0.0f);
	}

	Sphere::Sphere(Vec3 Center, float Radius)
	{
		Set(Center, Radius);
	}

	Sphere::Sphere(const Sphere& SphereShape)
	{
		this->Radius = SphereShape.Radius;
		this->CenterPosition = SphereShape.CenterPosition;
	}

	Sphere& Sphere::operator=(const Sphere& SphereShape)
	{
		this->Radius = SphereShape.Radius;
		this->CenterPosition = SphereShape.CenterPosition;

		return *this;
	}

	bool Sphere::NarrowPhaseCollisionTest(const Sphere& Object2)
	{
		Vec3 D = this->CenterPosition - Object2.CenterPosition;
		float D_sq = DotProduct(D, D);

		float Sum_Radius = this->Radius + Object2.Radius;

		return (D_sq <= (Sum_Radius * Sum_Radius));
	}

	void Sphere::Set(Vec3 Center, float Radius)
	{
		this->Radius = Radius;
		this->CenterPosition = Center;
	}
}