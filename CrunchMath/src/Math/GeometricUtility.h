#pragma once
#include "OBB.h"
#include "Sphere.h"

namespace CrunchMath {
	
	static bool IntersectTest(Sphere& SphereObject, OBB& OBBObject)
	{
		Vec3 P = OBBObject.ClosestPointOBBPt(SphereObject.CenterPosition);

		return (DotProduct(P - SphereObject.CenterPosition, P - SphereObject.CenterPosition) <=
			                                    (SphereObject.Radius * SphereObject.Radius));
	}
}