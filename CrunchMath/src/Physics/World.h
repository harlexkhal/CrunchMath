#pragma once
#include "Collisions.h"

namespace CrunchMath {

	class World
	{
	public:
		World(int bodycount, Vec3 gravity);
		World(Vec3 gravity);
		//~World() { delete[] Stack; };

		bool empty()
		{
			if (Size == 0)
				return true;

			return false;
		}

		Body* CreateBody(Shape* primitive);
		void Step(float dt);
	private:
		Body* Stack;
		int Index = 0;
		int Size = 0;
		Vec3 Gravity = Vec3(0.0f, 0.0f, 0.0f);

		/** Holds the maximum number of Contacts. */
		const static unsigned MaxContacts = 5000;

		/** Holds the array of Contacts. */
		CrunchMath::Contact Contacts[MaxContacts];

		/** Holds the collision data structure for collision detection. */
		CrunchMath::CollisionData CData;

		/** Holds the contact Resolver. */
		CrunchMath::ContactResolver Resolver = MaxContacts;

	};
}