#pragma once
#include "Vec3.h"
#include "Vec4.h"
#include "Mat3x3.h"
#include "Mat4x4.h"

namespace CrunchMath {

	struct OBB
	{
		float Center[3];
		float OrientationMatrix[3][3];
		float HalfExtent[3];

		OBB();

		OBB(const Vec3& Center, const Mat3x3& Orientation, const Vec3& HalfExtent);

		OBB(const OBB& OrientedBox);

		OBB& operator=(const OBB& OrientedBox);

		Vec3 ClosestPointOBBPt(const Vec3& Point) const;

		bool NarrowPhaseCollisionTest(const OBB& OrientedBox);

		bool BroadPhaseCollisionTest(const OBB& OrientedBox);

		void Set(const Vec3& Center, const Mat3x3& Orientation, const Vec3& HalfExtent);
	};
}
