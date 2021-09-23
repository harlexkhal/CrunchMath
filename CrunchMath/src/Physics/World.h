#pragma once
#include "Collisions.h"

namespace CrunchMath {

	class World
	{
	public:
		World(Vec3 gravity);
		~World();

		bool Empty()
		{
			for (int i = 0; i < MaxNumberOfBodies; i++)
			{
				if (FreeStack[i] == false)
					return false;
			}

			return true;
		}

		Body* CreateBody(Shape* primitive);
		void Step(float dt);
	private:

		World* m_pNext;

		const static unsigned int MaxNumberOfBodies = 100;
		Body Stack[MaxNumberOfBodies];
		bool FreeStack[MaxNumberOfBodies];
		unsigned int Index = 0;
		unsigned int Size = MaxNumberOfBodies;
		Vec3 Gravity = Vec3(0.0f, 0.0f, 0.0f);

		/** Holds the maximum number of Contacts. */
		const static unsigned MaxContacts = 20000;

		/** Holds the array of Contacts. */
		CrunchMath::Contact* Contacts;

		/** Holds the collision data structure for collision detection. */
		CrunchMath::CollisionData CData;

		/** Holds the contact Resolver. */
		CrunchMath::ContactResolver Resolver = MaxContacts;

	};
}