#pragma once
#include "Math/Mat4x4.h"

enum class CMShape
{
	Box,
	Sphere
};

class Body
{
public:
	Body(CMShape type);
	~Body();

	void Integrate(float dt);

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
	void SetAwake(bool a);

	CrunchMath::Vec3 GetPosition() const;
	const CrunchMath::Mat4x4& GetTransform() const;

	void AddVelocity(float x, float y, float z);

private:
	void CalculateTransform();
	void CalculateInertiaTensor();

	float Mass, InverseMass;
	CrunchMath::Vec3 Size;
	CrunchMath::Vec3 HalfExtent;

	float LinearDamping, AngularDamping;

	CrunchMath::Vec3 Position;
	CrunchMath::Vec3 Rotation;

	CrunchMath::Vec3 Velocity;
	CrunchMath::Vec3 AngularVelocity;

	//Just using a 9 slots (3x3)...until a 3x3 matrix is available in the math library
	CrunchMath::Mat4x4 InertiaTensorObject;
	CrunchMath::Mat4x4 InertiaTensorWorld, InverseInertiaTensorWorld;

	CrunchMath::Vec3 Acceleration;
	CrunchMath::Vec3 AngularAcceleration;
	CrunchMath::Vec3 LastFrameAcceleration;

	CrunchMath::Vec3 ForceAcc;
	CrunchMath::Vec3 TorqueAcc;

	CrunchMath::Mat4x4 TransformMatrix;

	CMShape Type;
	bool MassExist = false;
	bool SizeExist = false;
	bool Awake = true;

	Body* pNext = nullptr;
	Body* pPrev = nullptr;

	friend class World;
};