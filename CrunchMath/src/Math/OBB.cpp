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

	OBB::OBB(const Vec3& Center, const Mat3x3& Orientation, const Vec3& HalfExtent)
	{
		Set(Center, Orientation, HalfExtent);
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

			//Projecting the point unto each axis with respect to its Orientation
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

	bool OBB::NarrowPhaseCollisionTest(const OBB& OrientedBox)
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

	bool OBB::BroadPhaseCollisionTest(const OBB& OrientedBox)
	{
		//---None Exact OBB-OBB intersection Test---

		unsigned int NumAxis = 3;

		float this_Radius;
		float OrientedBox_Radius;

		Vec3 this_Center(Center[0], Center[1], Center[2]);
		Vec3 OrientedBox_Center(OrientedBox.Center[0], OrientedBox.Center[1], OrientedBox.Center[2]);

		float RM[3][3];
		float absRM[3][3];
		float hold = 0.0f;

		for (int h = 0; h < NumAxis; h++)
		{
			for (int i = 0; i < NumAxis; i++)
			{
				for (int j = 0; j < NumAxis; j++)
				{
					hold += (OrientationMatrix[j][h] * OrientedBox.OrientationMatrix[i][j]);
				}

				RM[i][h] = hold;
				hold = 0.0f;
			}
		}
		
		//Compute Translation T
		Vec3 Tvec = OrientedBox_Center - this_Center;
		//Bringing the Translation into this->OrientedBox object co-ordinate frame
		Tvec = Vec3(DotProduct(Vec3(OrientationMatrix[0][0], OrientationMatrix[1][0], OrientationMatrix[2][0]), Tvec),
			        DotProduct(Vec3(OrientationMatrix[0][1], OrientationMatrix[1][1], OrientationMatrix[2][1]), Tvec),
			        DotProduct(Vec3(OrientationMatrix[0][2], OrientationMatrix[1][2], OrientationMatrix[2][2]), Tvec));
		float T[3] = { Tvec.x, Tvec.y, Tvec.z };
		
		 /*Compute common subexpressions. Add in an epsilon term to
           counteract arithmetic errors when two edges are parallel and
           their cross product is (near) null
		   such errors normally occur in 3D not 2D*/
		for (int i = 0; i < NumAxis; i++)
		{
			for (int j = 0; j < NumAxis; j++)
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

			float sep = std::abs(T[i]) - this_Radius - OrientedBox_Radius;
			if (sep > 0.0f)
				return false;
		}

		// Test axes L = B0, L = B1, L = B2
		for (int i = 0; i < NumAxis; i++)
		{
			this_Radius = (HalfExtent[0] * absRM[0][i] +
				(HalfExtent[1] * absRM[1][i]) +
				(HalfExtent[2] * absRM[2][i]));

			OrientedBox_Radius = OrientedBox.HalfExtent[i];
			float sep = std::abs(T[0] * RM[0][i] + T[1] * RM[1][i] + T[2] * RM[2][i]) - this_Radius - OrientedBox_Radius;
			if (sep > 0.0f)
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

	void OBB::Set(const Vec3& Center, const Mat3x3& Orientaion, const Vec3& HalfExtent)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				//Transposed
				OrientationMatrix[j][i] = Orientaion.Matrix[i][j];
			}
		}

		this->Center[0] = Center.x;
		this->Center[1] = Center.y;
		this->Center[2] = Center.z;

		this->HalfExtent[0] = HalfExtent.x;
		this->HalfExtent[1] = HalfExtent.y;
		this->HalfExtent[2] = HalfExtent.z;
	}
}