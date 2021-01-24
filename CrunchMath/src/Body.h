#pragma once
#include "Math/Mat4x4.h"

class Body
{
public:
	Body();
	~Body();

	void Integrate(float dt);
	void CalculateTransform();

	void SetMass(float p_Mass);
	void SetSize(float x, float y, float z);
	void SetLinearDamping(float p_LinearDamping);
	void SetAngularDamping(float p_AngularDamping);
	void SetPosition(float x, float y, float z);
	void SetOrientation(float Angle, float Ax, float Ay, float Az);
	void SetVelocity(float x, float y, float z);
	void SetAngularVelocity(float x, float y, float z);
	void SetInertiaTensor();
	void SetAcceleration(float x, float y, float z);

	void AddVelocity(float x, float y, float z);
	CrunchMath::Vec3 GetPosition() const;
private:

	float Mass, InverseMass;
	CrunchMath::Vec3 Size;

	float LinearDamping, AngularDamping;

	CrunchMath::Vec3 Position;
	CrunchMath::Vec3 Rotation;

	CrunchMath::Vec3 Velocity;
	CrunchMath::Vec3 AngularVelocity;
	//Just using a 9 slots (3x3)...until a 3x3 matrix is available in the math library
	CrunchMath::Mat4x4 InertialTensorWorld, InverseInertialTensorWorld;

	CrunchMath::Vec3 Acceleration;
	CrunchMath::Vec3 LastFrameAcceleration;

	CrunchMath::Vec3 ForceAcc;
	CrunchMath::Vec3 TorqueAcc;

	CrunchMath::Mat4x4 TransformMatrix;

	CrunchMath::Vec3 AngularAcceleration;

	Body* pNext = nullptr;
	Body* pPrev = nullptr;

	friend class World;
};