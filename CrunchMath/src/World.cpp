#include "World.h"

World::World(float x, float y, float z)
{
	Gravity = CrunchMath::Vec3(x, y, z);
}

World::~World()
{
}

Body* World::CreateBody()
{
	if (FirstRegistered == nullptr)
	{
		FirstRegistered = new Body();
		FirstRegistered->pPrev = nullptr;
		FirstRegistered->pNext = nullptr;
		LastRegistered = FirstRegistered;

		return LastRegistered;
	}

	Body* NewBody = new Body();
	LastRegistered->pNext = NewBody;
	NewBody->pPrev = LastRegistered;
	NewBody->pNext = nullptr;
	LastRegistered = NewBody;

	return LastRegistered;
}

void World::Step(float dt)
{
	LastRegistered = FirstRegistered;

	while (LastRegistered != nullptr)
	{
		LastRegistered->ForceAcc += Gravity * LastRegistered->Mass;
		LastRegistered->Integrate(dt);

		//Temporary code
		if (LastRegistered->Position.y <= -0.7f)
			LastRegistered->AddVelocity(0, 0.5f, 0.0f);

		LastRegistered = LastRegistered->pNext;
	}
}
