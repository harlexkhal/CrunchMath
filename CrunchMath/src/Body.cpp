#include "Body.h"

Body::Body()
{
	Mass = 0.0f;
	InverseMass = 0.0f;
	
	LinearDamping = 1.0f;
	AngularDamping = 1.0f;
	TransformMatrix = CrunchMath::Mat4x4(1.0f);
	InertialTensorWorld = CrunchMath::Mat4x4(0.0f);
}

Body::~Body()
{
}

void Body::Integrate(float dt)
{
	LastFrameAcceleration = Acceleration;
	LastFrameAcceleration += ForceAcc * InverseMass;

	AngularAcceleration = InverseInertialTensorWorld * TorqueAcc;

	Velocity += LastFrameAcceleration * dt;
	AngularVelocity += AngularAcceleration * dt;

	Position *= pow(LinearDamping, dt);
	Rotation *= pow(AngularDamping, dt);

	Position += Velocity * dt;
	Rotation += AngularVelocity * dt;

	CalculateTransform();

	//Clear Force and Torque Accumulators
	ForceAcc = CrunchMath::Vec3();
	TorqueAcc = CrunchMath::Vec3();
}

void Body::CalculateTransform()
{
	CrunchMath::Mat4x4 Rx(1.0f), Ry(1.0f), Rz(1.0f);
	Rx.Rotate(CrunchMath::Vec3(1, 0, 0), CrunchMath::Radian(Rotation.x));
	Ry.Rotate(CrunchMath::Vec3(0, 1, 0), CrunchMath::Radian(Rotation.y));
	Rz.Rotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::Radian(Rotation.z));
	Rx *= Ry * Rz;

	TransformMatrix *= Rx;
	TransformMatrix.Translate(Position);
}

void Body::SetMass(float p_Mass)
{
	if (p_Mass <= 0.0f)
		return;

	Mass = p_Mass;
	InverseMass = (1.0f / p_Mass);
}

void Body::SetSize(float x, float y, float z)
{
	Size = CrunchMath::Vec3(x, y, z);
}

void Body::SetLinearDamping(float p_LinearDamping)
{
	LinearDamping = p_LinearDamping;
}

void Body::SetAngularDamping(float p_AngularDamping)
{
	AngularDamping = p_AngularDamping;
}

void Body::SetPosition(float x, float y, float z)
{
	Position = CrunchMath::Vec3(x, y, z);
	TransformMatrix.Translate(Position);
}

void Body::SetOrientation(float Angle, float Ax, float Ay, float Az)
{
	if (Ax > 0.0f)
	{
		Rotation.x = Angle;
	}

	if (Ay > 0.0f)
	{
		Rotation.y = Angle;
	}

	if (Az > 0.0f)
	{
		Rotation.z = Angle;
	}

	TransformMatrix.Rotate(CrunchMath::Vec3(Ax, Ay, Az), CrunchMath::Radian(Angle));
}

void Body::SetVelocity(float x, float y, float z)
{
	Velocity = CrunchMath::Vec3(x, y, z);
}

void Body::SetAngularVelocity(float x, float y, float z)
{
	AngularVelocity = CrunchMath::Vec3(x, y, z);
}

void Body::SetAcceleration(float x, float y, float z)
{
	Acceleration = CrunchMath::Vec3(x, y, z);
}

void Body::AddVelocity(float x, float y, float z)
{
	Velocity += CrunchMath::Vec3(x, y, z);
}

CrunchMath::Vec3 Body::GetPosition() const
{
	return Position;
}

void Body::SetInertiaTensor()
{

}