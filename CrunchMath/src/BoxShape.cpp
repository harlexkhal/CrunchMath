#include "BoxShape.h"

BoxShape::BoxShape(float Sizex, float Sizey, float Sizez)
{
	CreateBox(CrunchMath::Vec3(Sizex, Sizey, Sizez));
}

BoxShape::~BoxShape()
{
}

void BoxShape::CreateBox(CrunchMath::Vec3 Size)
{
	HalfExtent = Size * 0.5f;
}