#pragma once
#include "../Math/Mat3x3.h"
#include "../Math/Mat4x4.h"
#include "Core.h"

namespace CrunchMath {

    class Body
    {
    private:
        cpfloat inverseMass;
        Mat3x3 inverseInertiaTensor;

        cpfloat linearDamping;
        cpfloat angularDamping;

        Vec3 position;
        Quaternion orientation;

        Vec3 velocity;
        Vec3 rotation;

        Mat3x3 inverseInertiaTensorWorld;

        cpfloat motion;
        bool isAwake;
        bool canSleep;

        Mat4x4 transformMatrix;

        Vec3 forceAccum;
        Vec3 torqueAccum;

        Vec3 acceleration;
        Vec3 lastFrameAcceleration;

    public:
        void calculateDerivedData();
        void integrate(cpfloat duration);
        void setMass(const cpfloat mass);
        cpfloat getMass() const;
        cpfloat getInverseMass() const;
        void setInertiaTensor(const Mat3x3 &inertiaTensor);
        void getInertiaTensorWorld(Mat3x3& inertiaTensor) const;
        void getInverseInertiaTensorWorld(Mat3x3& inverseInertiaTensor) const;
        void setDamping(const cpfloat linearDamping, const cpfloat angularDamping);
        void setPosition(const Vec3 &position);
        void setPosition(const cpfloat x, const cpfloat y, const cpfloat z);
        void getPosition(Vec3 *position) const;
        Vec3 getPosition() const;
        void setOrientation(const Quaternion &orientation);
        void setOrientation(const cpfloat r, const cpfloat i, const cpfloat j, const cpfloat k);
        void getOrientation(Quaternion& orientation) const;
        void getOrientation(Mat3x3& matrix) const;
        Mat4x4 getTransform() const;
        void setVelocity(const cpfloat x, const cpfloat y, const cpfloat z);
        Vec3 getVelocity() const;
        void addVelocity(const Vec3 &deltaVelocity);
        void setRotation(const cpfloat x, const cpfloat y, const cpfloat z);
        Vec3 getRotation() const;
        void addRotation(const Vec3 &deltaRotation);

        bool getAwake() const
        {
            return isAwake;
        }
        void setAwake(const bool awake=true);
        bool getCanSleep() const
        {
            return canSleep;
        }

        void setCanSleep(const bool canSleep=true);
        Vec3 getLastFrameAcceleration() const;
        void clearAccumulators();
        void setAcceleration(const Vec3 &acceleration);

        void setInertiaTensorCoeffs(cpfloat ix, cpfloat iy, cpfloat iz, cpfloat ixy = 0, cpfloat ixz = 0, cpfloat iyz = 0);
        void setBlockInertiaTensor(const Vec3& halfSizes, cpfloat mass);
    };
}