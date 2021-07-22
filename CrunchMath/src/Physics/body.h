#pragma once
#include "core.h"

namespace CrunchPhysx {

    class RigidBody
    {
    protected:
        real inverseMass;
        Matrix3 inverseInertiaTensor;

        real linearDamping;
        real angularDamping;

        Vector3 position;
        Quaternion orientation;

        Vector3 velocity;
        Vector3 rotation;

        Matrix3 inverseInertiaTensorWorld;

        real motion;
        bool isAwake;
        bool canSleep;

        Matrix4 transformMatrix;

        Vector3 forceAccum;
        Vector3 torqueAccum;

        Vector3 acceleration;
        Vector3 lastFrameAcceleration;

    public:
        void calculateDerivedData();
        void integrate(real duration);
        void setMass(const real mass);
        real getMass() const;
        real getInverseMass() const;
        void setInertiaTensor(const Matrix3 &inertiaTensor);
        void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
        void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
        void setDamping(const real linearDamping, const real angularDamping);
        void setPosition(const Vector3 &position);
        void setPosition(const real x, const real y, const real z);
        void getPosition(Vector3 *position) const;
        Vector3 getPosition() const;
        void setOrientation(const Quaternion &orientation);
        void setOrientation(const real r, const real i, const real j, const real k);
        void getOrientation(Quaternion *orientation) const;
        void getOrientation(Matrix3 *matrix) const;
        void getOrientation(real matrix[9]) const;
        Matrix4 getTransform() const;
        void setVelocity(const real x, const real y, const real z);
        Vector3 getVelocity() const;
        void addVelocity(const Vector3 &deltaVelocity);
        void setRotation(const real x, const real y, const real z);
        Vector3 getRotation() const;
        void addRotation(const Vector3 &deltaRotation);

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
        Vector3 getLastFrameAcceleration() const;
        void clearAccumulators();
        void setAcceleration(const Vector3 &acceleration);
    };
}