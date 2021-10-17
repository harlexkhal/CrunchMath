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
		void SetIterations(uint32_t Position, uint32_t Velocity);
		void Step(float dt);
	private:
		//Constructor for children world blocks/nodes
		World(Vec3 gravity, bool parent);

		World* m_pNext;
		bool Parent;

		const static unsigned int MaxNumberOfBodies = 2;
		Body Stack[MaxNumberOfBodies];
		bool FreeStack[MaxNumberOfBodies];
		unsigned int Index = 0;
		unsigned int Size = MaxNumberOfBodies;
		Vec3 Gravity = Vec3(0.0f, 0.0f, 0.0f);

		/** Holds the maximum number of Contacts. */
		const static unsigned MaxContacts = 1000;

		/** Holds the array of Contacts. */
		CrunchMath::Contact Contacts[MaxContacts];

		/** Holds the collision data structure for collision detection. */
		CrunchMath::CollisionData CData;

		/** Holds the contact Resolver. */
		CrunchMath::ContactResolver Resolver;
	};
}