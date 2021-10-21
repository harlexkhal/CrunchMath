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

		Body* CreateBody(cmShape* primitive);
		void SetIterations(uint32_t Position, uint32_t Velocity);
		void Step(float dt);
	private:
		//Constructor for children world blocks/nodes
		World(Vec3 gravity, bool parent);

		World* m_pNext;
		bool Parent;

		//max number of body handled...if you a user exceeds this limit.
		//a child world is created in form of a linked list connecting with its parents.
		//this is done so to reduce memory fragmentation in creating and deleting rigid bodies.
		//since rigid bodies are always allocated on heap. so we just prepare a block of memory
		//for set of rigid bodies. you know what i mean...:-)
		const static unsigned int MaxNumberOfBodies = 100;

		Body Stack[MaxNumberOfBodies];
		bool FreeStack[MaxNumberOfBodies];
		unsigned int Index = 0;
		unsigned int Size = MaxNumberOfBodies;
		Vec3 Gravity = Vec3(0.0f, 0.0f, 0.0f);

		/** Holds the maximum number of Contacts. */
		const static unsigned MaxContacts = 5000;

		/** Holds the array of Contacts. */
		CrunchMath::Contact Contacts[MaxContacts];

		/** Holds the collision data structure for collision detection. */
		CrunchMath::CollisionData CData;

		/** Holds the contact Resolver. */
		CrunchMath::ContactResolver Resolver;
	};
}