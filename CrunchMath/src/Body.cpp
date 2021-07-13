#include "Body.h"

Body::Body(CMShape type)
{
	Type = type;

	Mass = 0.0f;
	InverseMass = 0.0f;
	
	LinearDamping = 1.0f;
	AngularDamping = 1.0f;
	TransformMatrix = CrunchMath::Mat4x4(1.0f);
	InertiaTensorObject = CrunchMath::Mat4x4(1.0f);
	InertiaTensorWorld = CrunchMath::Mat4x4(1.0f);
}

Body::~Body()
{
}

void Body::Integrate(float dt)
{
	if (!Awake)
		return;

	LastFrameAcceleration = Acceleration;
	LastFrameAcceleration += ForceAcc * InverseMass;
	
	AngularAcceleration = InverseInertiaTensorWorld * TorqueAcc;

	Velocity += LastFrameAcceleration * dt;
	AngularVelocity += AngularAcceleration * dt;

	Position *= pow(LinearDamping, dt);
	Rotation *= pow(AngularDamping, dt);

	Position += Velocity * dt;
	Rotation += AngularVelocity * dt;

	CalculateTransform();
	CalculateInertiaTensor();

	//Clear Force and Torque Accumulators
	ForceAcc = CrunchMath::Vec3();
	TorqueAcc = CrunchMath::Vec3();
}

void Body::CalculateTransform()
{
	//usually should be done in the order of Scale => Rotate => Translate
	//therefore since matrix multiplication operates from left to right we 
	//do => Translate x Rotate x Scale

	//Reset Transform matrix for new Transform
	TransformMatrix = CrunchMath::Mat4x4(1.0f);

	TransformMatrix.Translate(Position);

	CrunchMath::Mat4x4 Rx(1.0f), Ry(1.0f), Rz(1.0f);
	Rx.Rotate(CrunchMath::Vec3(1, 0, 0), CrunchMath::Radian(Rotation.x));
	Ry.Rotate(CrunchMath::Vec3(0, 1, 0), CrunchMath::Radian(Rotation.y));
	Rz.Rotate(CrunchMath::Vec3(0, 0, 1), CrunchMath::Radian(Rotation.z));
	Rx *= Ry * Rz;

	TransformMatrix *= Rx;

	TransformMatrix.Scale(CrunchMath::Vec3(Size.x, Size.y, Size.z));
}

void Body::SetMass(float p_Mass)
{
	if (p_Mass <= 0.0f)
		return;

	Mass = p_Mass;
	InverseMass = (1.0f / p_Mass);
	MassExist = true;
}

void Body::SetSize(float x, float y, float z)
{
	if (z <= 0)
		z = 1.0f;

	Size = CrunchMath::Vec3(x, y, z);
	HalfExtent = Size * 0.5f;
	SizeExist = true;
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

void Body::SetInertiaTensor()
{
	assert(MassExist && SizeExist);

	switch (Type)
	{
	case CMShape::Box :
		CrunchMath::Vec3 Moments = CrunchMath::Vec3(
			(
				(1.0f / 12.0f) * Mass * ( (HalfExtent.y * HalfExtent.y) + (HalfExtent.z * HalfExtent.z) )
				),

			(
				(1.0f / 12.0f) * Mass * ( (HalfExtent.x * HalfExtent.x) + (HalfExtent.z * HalfExtent.z) )
				),

			(
				(1.0f / 12.0f) * Mass * ( (HalfExtent.x * HalfExtent.x) + (HalfExtent.y * HalfExtent.y) )
				)
		);

		InertiaTensorObject.InsertDiagonal(Moments);
		break;
	}
}

void Body::SetAcceleration(float x, float y, float z)
{
	Acceleration = CrunchMath::Vec3(x, y, z);
}

void Body::SetAwake(bool a)
{
	//clear All Velocity Accumulations acquired during sleep
	Velocity = CrunchMath::Vec3(0.0f, 0.0f, 0.0f);
	AngularVelocity = CrunchMath::Vec3(0.0f, 0.0f, 0.0f);

	Awake = true;
}

void Body::AddVelocity(float x, float y, float z)
{
	Velocity += CrunchMath::Vec3(x, y, z);
}

CrunchMath::Vec3 Body::GetPosition() const
{
	return Position;
}

const CrunchMath::Mat4x4& Body::GetTransform() const
{
	return TransformMatrix;
}

void Body::CalculateInertiaTensor()
{
	//worldInertiaTensor = transpose(inverse(localToWorld)) * localInertiaTensor * inverse(localToWorld);
	CrunchMath::Mat4x4 InverseTransform = CrunchMath::Invert(TransformMatrix); 
	InertiaTensorWorld = InverseTransform.Transpose() * InertiaTensorObject * InverseTransform.Transpose();

	//Making the Translation Vector insignificant cos its not needed to calculate angular speed. hence Mat3x3 needs to be used...=>TODO
	InertiaTensorWorld.Translate(CrunchMath::Vec3(0.0f, 0.0f, 0.0f));

	InverseInertiaTensorWorld = CrunchMath::Invert(InertiaTensorWorld);
}