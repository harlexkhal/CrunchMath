#pragma once
#include "Shape.h"

class BoxShape : public Shape
{
public:
	BoxShape(float Sizex, float Sizey, float Sizez);
	~BoxShape();

	virtual ShapeType GetType() const override { return this->Type; }
private:
	virtual void CreateBox(CrunchMath::Vec3 Size) override;
};