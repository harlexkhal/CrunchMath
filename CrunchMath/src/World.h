#pragma once
#include "Body.h"

class World
{
public:
	World(float x, float y, float z);
	~World();

	Body* CreateBody(CMShape type);
	void Step(float dt);

private:
	Body* FirstRegistered = nullptr;
	Body* LastRegistered = nullptr;
	CrunchMath::Vec3 Gravity;
};