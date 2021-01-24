#pragma once
#include "CrunchMath.h"

enum class ShapeType
{
	Box,
	Sphere
};

class Shape
{
public:
	Shape() {};
	~Shape() {};

	virtual void CreateBox(CrunchMath::Vec3 Size) = 0;
	virtual void CreateSphere(float Diameter) = 0;
	virtual ShapeType GetType() const = 0;

protected:
	ShapeType Type;
	CrunchMath::Vec3 HalfExtent;
};