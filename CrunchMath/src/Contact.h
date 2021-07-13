#pragma once
#include "Body.h"

class Contact
{
public:
    Body* body[2];

    CrunchMath::Vec3 ContactPoint;

    CrunchMath::Vec3 ContactNormal;

    float Penetration;
};