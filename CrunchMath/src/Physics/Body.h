#pragma once
#include "../Math/Mat3x3.h"
#include "../Math/Mat4x4.h"

namespace CrunchMath {

    class Shape;
    
    class Body
    {
        friend class World;
    public:
        Body();
        Body(const Body& copybody);
        Body& operator=(const Body& copybody);
        ~Body(){};

        void CalculateDerivedData();
        void Integrate(float duration);
        void SetMass(const float mass);
        float GetMass() const;
        float GetInverseMass() const;
        void SetInertiaTensor(const Mat3x3 &inertiaTensor);
        void GetInertiaTensorWorld(Mat3x3& inertiaTensor) const;
        void GetInverseInertiaTensorWorld(Mat3x3& InverseInertiaTensor) const;
        void SetDamping(const float LinearDamping, const float AngularDamping);
        void SetPosition(const Vec3 &Position);
        void SetPosition(const float x, const float y, const float z);
        void GetPosition(Vec3& Position) const;
        Vec3 GetPosition() const;
        void SetOrientation(const Quaternion &Orientation);
        void SetOrientation(const float r, const float i, const float j, const float k);
        void GetOrientation(Quaternion& Orientation) const;
        void GetOrientation(Mat3x3& matrix) const;
        Mat4x4 GetTransform() const;
        void SetVelocity(const float x, const float y, const float z);
        Vec3 GetVelocity() const;
        void AddVelocity(const Vec3 &deltaVelocity);
        void SetRotation(const float x, const float y, const float z);
        Vec3 GetRotation() const;
        void AddRotation(const Vec3 &deltaRotation);

        bool GetAwake() const;
        void SetAwake(const bool awake=true);
 
        Vec3 GetLastFrameAcceleration() const;
        void ClearAccumulators();
        void SetAcceleration(const Vec3 &Acceleration);

        void SetInertiaTensorCoeffs(float ix, float iy, float iz, float ixy = 0, float ixz = 0, float iyz = 0);
        void SetBlockInertiaTensor(const Vec3& HalfSizes, float mass);

    private:
        void Copy(const Body& copybody);

        float InverseMass;
        Mat3x3 InverseInertiaTensor;

        float LinearDamping;
        float AngularDamping;

        Vec3 Position;
        Quaternion Orientation;

        Vec3 Velocity;
        Vec3 Rotation;

        Mat3x3 InverseInertiaTensorWorld;

        float Motion;
        bool IsAwake;
        bool CanSleep;

        Mat4x4 TransformMatrix;

        Vec3 ForceAccumulation;
        Vec3 TorqueAccumulation;

        Vec3 Acceleration;
        Vec3 LastFrameAcceleration;

        Shape* Primitive = nullptr;
        Body* m_pNext;
    };

    /*
      Holds the value for energy under which a body will be put to
      sleep. This is a global value for the whole solution.  By
      default it is 0.1, which is fine for simulation when gravity is
      about 20 units per second squared, masses are about one, and
      other forces are around that of gravity. It may need tweaking
      if your simulation is drastically different to this.

      Defined in Body.cpp***
    */
    extern float SleepEpsilon;
}