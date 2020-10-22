#include "OBB.h"

namespace CrunchMath {

	OBB::OBB()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				OrientationMatrix[i][j] = 0.0f;

				if (i == 0)
				{
					HalfExtent[j] = 0.0f;
					Center[j] = 0.0f;
				}
			}
		}
	}

	OBB::OBB(const Vec3& Center, const Vec4& RotationAxis, const Vec3& HalfExtent)
	{
		Set(Center, RotationAxis, HalfExtent);
	}

	OBB::OBB(const OBB& OrientedBox)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				OrientationMatrix[i][j] = OrientedBox.OrientationMatrix[i][j];

				if (i == 0)
				{
					HalfExtent[j] = OrientedBox.HalfExtent[j];
					Center[j] = OrientedBox.Center[j];
				}
			}
		}
	}

	OBB& OBB::operator=(const OBB& OrientedBox)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				OrientationMatrix[i][j] = OrientedBox.OrientationMatrix[i][j];

				if (i == 0)
				{
					HalfExtent[j] = OrientedBox.HalfExtent[j];
					Center[j] = OrientedBox.Center[j];
				}
			}
		}

		return *this;
	}

	Vec3 OBB::ClosestPointOBBPt(const Vec3& Point) const
	{
		Vec3 d;
		d.x = Point.x - Center[0];
		d.y = Point.y - Center[1];
		d.z = Point.z - Center[2];

		Vec3 ClosestPointOnOBB;
		ClosestPointOnOBB.x = Center[0];
		ClosestPointOnOBB.y = Center[1];
		ClosestPointOnOBB.z = Center[2];

		for (int i = 0; i < 3; i++)
		{
			int j = 0;
			//Orientation of the x-axis[0][0 <-to-> 2] , y-axis[1][0 <-to-> 2] and z-axis[2][0 <-to-> 2]
			Vec3 Orient(OrientationMatrix[i][j++], OrientationMatrix[i][j++], OrientationMatrix[i][j]);

			//Projecting the point unto each axis with respect to its orientation
			float Dist = DotProduct(Orient, d);

			if (Dist > HalfExtent[i])
			{
				Dist = HalfExtent[i];
			}

			else if (Dist < -HalfExtent[i])
			{
				Dist = -HalfExtent[i];
			}

			//Computing ClosestPoint from object space to world space
			ClosestPointOnOBB += Orient * Dist;
		}

		return ClosestPointOnOBB;
	}

	bool OBB::IntersectTest(const OBB& OrientedBox)
	{
		Vec3 C1(Center[0], Center[1], Center[2]);
		Vec3 C2(OrientedBox.Center[0], OrientedBox.Center[1], OrientedBox.Center[2]);

		//To get the radius of OrientedBox with respect to the closest point on OrientedBox(var)->OBB to this->OBB center
		Vec3 P2 = OrientedBox.ClosestPointOBBPt(C1);
		//Distance between C2 [to] P2 gives the Radius of the OrientedBox->OBB
		/*using Distance() func not recommended because of Sqrt operation >>> slows down program. 
		  never use a sqrt() if you won't need it. since all we want to do is compare Distance's*/
		//float Radius2 = Distance(C2, P2); //for better performance don't use Distance(C2, P2) to compute Radius2
		float Radius2 = DotProduct(C2 - P2, C2 - P2); //Gives the Squared Distance C2 [to] P2

		//Getting closest  point on this->OBB to closest radius extent point on OrientedBox->OBB
		Vec3 P1 = this->ClosestPointOBBPt(P2);

		//Distance between closest point P1 [to] C2(OrientedBox->OBB center point)
		/*using Distance() func not recommended because of Sqrt operation >>> slows down program.
		never use a sqrt() if you won't need it. since all we want to do is compare Distance's*/
		//float D = Distance(P1, C2); //for better performance don't use Distance(P1, C2) to compute D
		float D = DotProduct(P1 - C2, P1 - C2); //Gives the Squared Distance P1 [to] C2

		/*Note : if using Squared Distance(DotProduct func) both Radius2 and D should be in Squared Distance,
		if using actual Distance(Distance func) both Radius2 and D should be in Actual Distance. any other way
		makes the whole query read wrongly*/

		//Reports as intersecting if Radius(C2-P2) becomes > or = the Distance (P1-C2)
		return (Radius2 >= D);
	}

	bool OBB::ALTIntersectTest(const OBB& OrientedBox)
	{
		//---None Exact OBB-OBB intersection Test---

		//Checks to see if to do the test in 2D or 3D for a little bit accuracy in its unreliable intersection tests :-{
		unsigned int NumAxis = 3;
		if (HalfExtent[2] <= 0.0f && OrientedBox.HalfExtent[2] <= 0.0f)
			NumAxis = 2;

		float this_Radius;
		float OrientedBox_Radius;

		Vec3 this_Center(Center[0], Center[1], Center[2]);
		Vec3 OrientedBox_Center(OrientedBox.Center[0], OrientedBox.Center[1], OrientedBox.Center[2]);

		float RM[3][3];
		float absRM[3][3];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Vec3 this_OrientationAxis(OrientationMatrix[i][0], 
					                      OrientationMatrix[i][1], 
					                      OrientationMatrix[i][2]);

				Vec3 OrientedBox_OrientationAxis(OrientedBox.OrientationMatrix[i][0], 
					                             OrientedBox.OrientationMatrix[i][1],
					                             OrientedBox.OrientationMatrix[i][2]);

				//Computing Rotation Matrix expressing OrientedBox in this->OrientedBox object co-ordinate frame
				RM[i][j] = DotProduct(this_OrientationAxis, OrientedBox_OrientationAxis);
			}
		}
		
		//Compute Translation T
		Vec3 Tvec = OrientedBox_Center - this_Center;
		//Bringing the Translation into this->OrientedBox object co-ordinate frame
		Tvec = Vec3(DotProduct(Vec3(OrientationMatrix[0][0], OrientationMatrix[0][1], OrientationMatrix[0][2]), Tvec),
			        DotProduct(Vec3(OrientationMatrix[1][0], OrientationMatrix[1][1], OrientationMatrix[1][2]), Tvec),
			        DotProduct(Vec3(OrientationMatrix[2][0], OrientationMatrix[2][1], OrientationMatrix[2][2]), Tvec));
		float T[3] = { Tvec.x, Tvec.y, Tvec.z };

		 /*Compute common subexpressions. Add in an epsilon term to
           counteract arithmetic errors when two edges are parallel and
           their cross product is (near) null
		   such errors normally occur in 3D not 2D*/
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				absRM[i][j] = std::abs(RM[i][j]) + FLT_EPSILON;
			}
		}

		/*Begin Testing Along all of the 15 different Axis 
		A reps this->OBB , B reps OrientedBox->OBB*/

		// Test axes L = A0, L = A1, L = A2
		for (int i = 0; i < NumAxis; i++)
		{
			this_Radius = HalfExtent[i];
			OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[i][0]) + 
				                 (OrientedBox.HalfExtent[1] * absRM[i][1]) +
				                 (OrientedBox.HalfExtent[2] * absRM[i][2]);

			if (std::abs(T[i]) > (this_Radius + OrientedBox_Radius))
				return false;
		}

		// Test axes L = B0, L = B1, L = B2
		for (int i = 0; i < NumAxis; i++)
		{
			this_Radius = (HalfExtent[0] * absRM[0][i] +
				(HalfExtent[1] * absRM[1][i]) +
				(HalfExtent[2] * absRM[2][i]));

			OrientedBox_Radius = OrientedBox.HalfExtent[i];

			if (std::abs(T[0] * RM[0][i] + T[1] * RM[1][i] + T[2] * RM[2][i]) > (this_Radius + OrientedBox_Radius))
				return false;
		}

		// Test axis L = A0 x B0
		this_Radius = (HalfExtent[1] * absRM[2][0]) + (HalfExtent[2] * absRM[1][0]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[1] * absRM[0][2]) + (OrientedBox.HalfExtent[2] * absRM[0][1]);
		if (std::abs((T[2] * RM[1][0]) - (T[1] * RM[2][0])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A0 x B1
		this_Radius = (HalfExtent[1] * absRM[2][1]) + (HalfExtent[2] * absRM[1][1]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[0][2]) + (OrientedBox.HalfExtent[2] * absRM[0][0]);
		if (std::abs((T[2] * RM[1][1]) - (T[1] * RM[2][1])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A0 x B2
		this_Radius = (HalfExtent[1] * absRM[2][2]) + (HalfExtent[2] * absRM[1][2]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[0][1]) + (OrientedBox.HalfExtent[1] * absRM[0][0]);
		if (std::abs((T[2] * RM[1][2]) - (T[1] * RM[2][2])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A1 x B0
		this_Radius = (HalfExtent[0] * absRM[2][0]) + (HalfExtent[2] * absRM[0][0]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[1] * absRM[1][2]) + (OrientedBox.HalfExtent[2] * absRM[1][1]);
		if (std::abs((T[0] * RM[2][0]) - (T[2] * RM[0][0])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A1 x B1
		this_Radius = (HalfExtent[0] * absRM[2][1]) + (HalfExtent[2] * absRM[0][1]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[1][2]) + (OrientedBox.HalfExtent[2] * absRM[1][0]);
		if (std::abs((T[0] * RM[2][1]) - (T[2] * RM[0][1])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A1 x B2
		this_Radius = (HalfExtent[0] * absRM[2][2]) + (HalfExtent[2] * absRM[0][2]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[1][1]) + (OrientedBox.HalfExtent[1] * absRM[1][0]);
		if (std::abs((T[0] * RM[2][2]) - (T[2] * RM[0][2])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A2 x B0
		this_Radius = (HalfExtent[0] * absRM[1][0]) + (HalfExtent[1] * absRM[0][0]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[1] * absRM[2][2]) + (OrientedBox.HalfExtent[2] * absRM[2][1]);
		if (std::abs((T[1] * RM[0][0]) - (T[0] * RM[1][0])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A2 x B1
		this_Radius = (HalfExtent[0] * absRM[1][1]) + (HalfExtent[1] * absRM[0][1]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[2][2]) + (OrientedBox.HalfExtent[2] * absRM[2][0]);
		if (std::abs((T[1] * RM[0][1]) - (T[0] * RM[1][1])) > this_Radius + OrientedBox_Radius)
			return false;

		// Test axis L = A2 x B2
		this_Radius = (HalfExtent[0] * absRM[1][2]) + (HalfExtent[1] * absRM[0][2]);
		OrientedBox_Radius = (OrientedBox.HalfExtent[0] * absRM[2][1]) + (OrientedBox.HalfExtent[1] * absRM[2][0]);
		if (std::abs((T[1] * RM[0][2]) - (T[0] * RM[1][2])) > this_Radius + OrientedBox_Radius)
			return false;

		// Since no separating axis is found, the OBBs must be intersecting
		return true;
	}

	void OBB::Set(const Vec3& Center, const Vec4& RotationAxis, const Vec3& HalfExtent)
	{
		//Making sure its a unit Vector...
		assert(RotationAxis.x <= 1.0f && RotationAxis.x >= 0.0f);
		assert(RotationAxis.y <= 1.0f && RotationAxis.y >= 0.0f);
		assert(RotationAxis.z <= 1.0f && RotationAxis.z >= 0.0f);

		//making x-axis Sin and y-axis Cos---Switching axis to Switch from Left Handed to Right. Tested-withOpengl--WorkSpace Right Handed
		float c = sin(RotationAxis.a);
		float s = cos(RotationAxis.a);

		float a = 1 - c;

		//column vec 1...
		OrientationMatrix[0][0] = ((RotationAxis.x * RotationAxis.x) * a) + c;
		OrientationMatrix[0][1] = ((RotationAxis.x * RotationAxis.y) * a) + RotationAxis.z * s;
		OrientationMatrix[0][2] = ((RotationAxis.x * RotationAxis.z) * a) - RotationAxis.y * s;

		//column vec2...
		OrientationMatrix[1][0] = ((RotationAxis.x * RotationAxis.y) * a) - RotationAxis.z * s;
		OrientationMatrix[1][1] = ((RotationAxis.y * RotationAxis.y) * a) + c;
		OrientationMatrix[1][2] = ((RotationAxis.y * RotationAxis.z) * a) + RotationAxis.x * s;

		//column vec3...
		OrientationMatrix[2][0] = ((RotationAxis.x * RotationAxis.z) * a) + RotationAxis.y * s;
		OrientationMatrix[2][1] = ((RotationAxis.y * RotationAxis.z) * a) - RotationAxis.x * s;
		OrientationMatrix[2][2] = ((RotationAxis.z * RotationAxis.z) * a) + c;

		this->Center[0] = Center.x;
		this->Center[1] = Center.y;
		this->Center[2] = Center.z;

		this->HalfExtent[0] = HalfExtent.x;
		this->HalfExtent[1] = HalfExtent.y;
		this->HalfExtent[2] = HalfExtent.z;
	}
}