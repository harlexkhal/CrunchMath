#include "World.h"

namespace CrunchMath {

	World::World(int bodycount, Vec3 gravity)
		:Size(bodycount),Gravity(gravity),Index(0)
	{
		Stack = new Body[bodycount];
		CData.ContactArray = Contacts;
	}

	World::World(Vec3 gravity)
		: Size(100), Gravity(gravity), Index(0)
	{
		Stack = new Body[100];
		CData.ContactArray = Contacts;
	}

	Body* World::CreateBody(Shape* primitive)
	{
		if (empty())
		{
			Stack[0].SetAcceleration(Gravity);
			Body* newbody = Stack + Index;
			newbody->Primitive = primitive;
			newbody->Primitive->SetBody(newbody);
			Index++;
		    return newbody;
		}

		else if (Index == Size)
		{
			assert(false);
		}

		else
		{
		   Stack[Index].SetAcceleration(Gravity);

		   Body* newbody = Stack + Index;
		   newbody->Primitive = primitive;
		   newbody->Primitive->SetBody(newbody);
		   Index++;
		   return newbody;
		}
	}

	void World::Step(float dt)
	{
		CData.Reset(MaxContacts);
		CData.Friction = 0.9f;
		CData.Restitution = 0.1f;

		for (int i = 0; i < Index; i++)
		{
			(Stack + i)->Integrate(dt);
		}

		for (int i = 0; i < Index; i++)
		{
			for (int j = i + 1; j < Index; j++)
			{
				CrunchMath::CollisionDetector::Collision(((Stack + i)->Primitive), ((Stack + j)->Primitive), &CData);
			}
		}

		for (int i = 0; i < Index; i++)
		{
			if ((Stack + i)->GetPosition().y <= -0.9f)
			{
				(Stack + i)->SetVelocity(0, 0, 0);
				(Stack + i)->SetRotation(0, 0, 0);
				(Stack + i)->SetPosition(0.0f, 0.9f, 0.0f);
			}
		}

		Resolver.ResolveContacts(CData.ContactArray, CData.ContactCount, dt);
	}
}