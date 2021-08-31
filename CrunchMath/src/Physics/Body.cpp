#include <float.h>
#include <memory.h>
#include <assert.h>
#include "Body.h"

namespace CrunchMath
{
    float SleepEpsilon = ((float)0.3);

    static inline void TransformInertiaTensor(Mat3x3& iitWorld, const Mat3x3& iitBody, const Mat4x4& rotmat)
    {
        Mat3x3 LocalToWorld(rotmat.GetColumnVector(0), rotmat.GetColumnVector(1), rotmat.GetColumnVector(2));
        Mat3x3 TransposeInverseLocalToWorld = Invert(LocalToWorld);
        TransposeInverseLocalToWorld.Transpose();

        iitWorld = TransposeInverseLocalToWorld * iitBody * Invert(LocalToWorld);
    }

    static inline void CalculateTransformMatrix(Mat4x4& TransformMatrix, const Vec3& Position, const Quaternion& Orientation)
    {
        TransformMatrix.Rotate(Orientation);
        TransformMatrix.Translate(Position);
    }

    Body::Body()
    {
        Position = Vec3(0.0f, 0.0f, 0.0f);
        Orientation = Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
        Velocity = Vec3(0.0f, 0.0f, 0.0f);
        SetDamping(0.9f, 0.9f);
        CalculateDerivedData();
    }

    Body::Body(const Body& copybody)
    {
        Copy(copybody);
    }

    Body& Body::operator=(const Body& copybody)
    {
        Copy(copybody);
        return *this;
    }

    void Body::CalculateDerivedData()
    {
        Orientation.Normalize();

        // Calculate the transform matrix for the body.
        CalculateTransformMatrix(TransformMatrix, Position, Orientation);

        // Calculate the inertiaTensor in world space.
        TransformInertiaTensor(InverseInertiaTensorWorld, InverseInertiaTensor, TransformMatrix);
    }

    void Body::Integrate(float duration)
    {
        if (!IsAwake) return;

        // Calculate linear Acceleration from force inputs.
        LastFrameAcceleration = Acceleration;
        LastFrameAcceleration += ForceAccumulation * InverseMass;

        // Calculate angular Acceleration from torque inputs.
        Vec3 angularAcceleration = InverseInertiaTensorWorld * TorqueAccumulation;

        // Adjust velocities
        // Update linear Velocity from both Acceleration and impulse.
        Velocity += LastFrameAcceleration * duration;

        // Update angular Velocity from both Acceleration and impulse.
        Rotation += angularAcceleration * duration;

        // Impose drag.
        Velocity *= powf(LinearDamping, duration);
        Rotation *= powf(AngularDamping, duration);

        // Adjust Positions
        // Update linear Position.
        Position += Velocity * duration;

        // Update angular Position. //To do=> Take away Quaternion later...
        Quaternion q(0, Vec3(Rotation * duration));
        q *= Orientation;
        Orientation.w += q.w * ((float)0.5);
        Orientation.x += q.x * ((float)0.5);
        Orientation.y += q.y * ((float)0.5);
        Orientation.z += q.z * ((float)0.5);

        // Normalise the Orientation, and update the matrices with the new
        // Position and Orientation
        CalculateDerivedData();

        // Clear accumulators.
        ClearAccumulators();

        // Update the kinetic energy store, and possibly put the body to
        // sleep.
        if (CanSleep) {
            float currentMotion = DotProduct(Velocity, Velocity) + DotProduct(Rotation, Rotation);

            float bias = powf(0.5, duration);
            Motion = bias * Motion + (1 - bias) * currentMotion;

            if (Motion < SleepEpsilon) SetAwake(false);
            else if (Motion > 10 * SleepEpsilon) Motion = 10 * SleepEpsilon;
        }
    }

    void Body::SetMass(const float mass)
    {
        if (mass <= 0.0f)
            InverseMass = 0.0f;
        else
            InverseMass = ((float)1.0) / mass;
    }

    float Body::GetMass() const
    {
        if (InverseMass == 0) {
            return FLT_MAX;
        }
        else {
            return ((float)1.0) / InverseMass;
        }
    }

    float Body::GetInverseMass() const
    {
        return InverseMass;
    }

    void Body::SetInertiaTensor(const Mat3x3& inertiaTensor)
    {
        InverseInertiaTensor = Invert(inertiaTensor);
    }

    void Body::GetInertiaTensorWorld(Mat3x3& inertiaTensor) const
    {
        inertiaTensor = Invert(InverseInertiaTensorWorld);
    }

    void Body::GetInverseInertiaTensorWorld(Mat3x3& InverseInertiaTensor) const
    {
        InverseInertiaTensor = InverseInertiaTensorWorld;
    }

    void Body::SetDamping(const float LinearDamping,
        const float AngularDamping)
    {
        this->LinearDamping = LinearDamping;
        this->AngularDamping = AngularDamping;
    }

    void Body::SetPosition(const Vec3& Position)
    {
        this->Position = Position;
    }

    void Body::SetPosition(const float x, const float y, const float z)
    {
        Position.x = x;
        Position.y = y;
        Position.z = z;
    }

    void Body::GetPosition(Vec3& Position) const
    {
        Position = this->Position;
    }

    Vec3 Body::GetPosition() const
    {
        return Position;
    }

    void Body::SetOrientation(const Quaternion& Orientation)
    {
        this->Orientation = Orientation;
        this->Orientation.Normalize();
    }

    void Body::SetOrientation(const float w, const float x, const float y, const float z)
    {
        Orientation.w = w;
        Orientation.x = x;
        Orientation.y = y;
        Orientation.z = z;
        Orientation.Normalize();
    }

    void Body::GetOrientation(Quaternion& Orientation) const
    {
        Orientation = this->Orientation;
    }

    void Body::GetOrientation(Mat3x3& matrix) const
    {
        matrix.Matrix[0][0] = TransformMatrix.Matrix[0][0];
        matrix.Matrix[0][1] = TransformMatrix.Matrix[0][1];
        matrix.Matrix[0][2] = TransformMatrix.Matrix[0][2];

        matrix.Matrix[1][0] = TransformMatrix.Matrix[1][0];
        matrix.Matrix[1][1] = TransformMatrix.Matrix[1][1];
        matrix.Matrix[1][2] = TransformMatrix.Matrix[1][2];

        matrix.Matrix[2][0] = TransformMatrix.Matrix[2][0];
        matrix.Matrix[2][1] = TransformMatrix.Matrix[2][1];
        matrix.Matrix[2][2] = TransformMatrix.Matrix[2][2];
    }

    Mat4x4 Body::GetTransform() const
    {
        return TransformMatrix;
    }

    void Body::SetVelocity(const float x, const float y, const float z)
    {
        Velocity.x = x;
        Velocity.y = y;
        Velocity.z = z;
    }

    Vec3 Body::GetVelocity() const
    {
        return Velocity;
    }

    void Body::AddVelocity(const Vec3& deltaVelocity)
    {
        Velocity += deltaVelocity;
    }

    void Body::SetRotation(const float x, const float y, const float z)
    {
        Rotation.x = x;
        Rotation.y = y;
        Rotation.z = z;
    }

    Vec3 Body::GetRotation() const
    {
        return Rotation;
    }

    void Body::AddRotation(const Vec3& deltaRotation)
    {
        Rotation += deltaRotation;
    }

    void Body::SetAwake(const bool awake)
    {
        if (awake)
        {
            IsAwake = true;

            // Add a bit of Motion to avoid it falling asleep immediately.
            Motion = SleepEpsilon * 2.0f;
        }

        else
        {
            IsAwake = false;
            Velocity = Vec3(0.0f, 0.0f, 0.0f);
            Rotation = Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    Vec3 Body::GetLastFrameAcceleration() const
    {
        return LastFrameAcceleration;
    }

    void Body::ClearAccumulators()
    {
        ForceAccumulation = Vec3(0.0f, 0.0f, 0.0f);
        TorqueAccumulation = Vec3(0.0f, 0.0f, 0.0f);
    }

    void Body::SetAcceleration(const Vec3& Acceleration)
    {
        Body::Acceleration = Acceleration;
    }

    bool Body::GetAwake() const
    {
        return IsAwake;
    }

    void Body::SetInertiaTensorCoeffs(float ix, float iy, float iz, float ixy, float ixz, float iyz)
    {
        Mat3x3 InertiaTensor(Vec3(ix, -ixy, -ixz), Vec3(-ixy, iy, -iyz), Vec3(-ixz, -iyz, iz));
        SetInertiaTensor(InertiaTensor);
    }

    void Body::SetBlockInertiaTensor(const Vec3& HalfSizes, float mass)
    {
        Vec3 squares = HalfSizes * HalfSizes;

        SetInertiaTensorCoeffs( 0.9f * mass * (squares.y + squares.z),
                                0.9f * mass * (squares.x + squares.z),
                                0.9f * mass * (squares.x + squares.y)
                              );
    }

    void Body::Copy(const Body& copybody)
    {
        InverseMass = copybody.InverseMass;
        InverseInertiaTensor = copybody.InverseInertiaTensor;
        LinearDamping = copybody.LinearDamping;
        AngularDamping = copybody.AngularDamping;
        Position = copybody.Position;
        Orientation = copybody.Orientation;
        Velocity = copybody.Velocity;
        Rotation = copybody.Rotation;
        InverseInertiaTensorWorld = copybody.InverseInertiaTensorWorld;
        Motion = copybody.Motion;
        IsAwake = copybody.IsAwake;
        CanSleep = copybody.CanSleep;
        TransformMatrix = copybody.TransformMatrix;
        ForceAccumulation = copybody.ForceAccumulation;
        TorqueAccumulation = copybody.TorqueAccumulation;
        Acceleration = copybody.Acceleration;
        LastFrameAcceleration = copybody.LastFrameAcceleration;
        Primitive = copybody.Primitive;
    }
}