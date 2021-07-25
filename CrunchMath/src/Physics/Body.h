#pragma once
#include "PhysxMath.h"
#include "Core.h"

namespace CrunchPhysx {

    class Body
    {
    private:
        cpfloat inverseMass;
        Matrix3 inverseInertiaTensor;

        cpfloat linearDamping;
        cpfloat angularDamping;

        Vector3 position;
        Quaternion orientation;

        Vector3 velocity;
        Vector3 rotation;

        Matrix3 inverseInertiaTensorWorld;

        cpfloat motion;
        bool isAwake;
        bool canSleep;

        Matrix4 transformMatrix;

        Vector3 forceAccum;
        Vector3 torqueAccum;

        Vector3 acceleration;
        Vector3 lastFrameAcceleration;

    public:
        void calculateDerivedData();
        void integrate(cpfloat duration);
        void setMass(const cpfloat mass);
        cpfloat getMass() const;
        cpfloat getInverseMass() const;
        void setInertiaTensor(const Matrix3 &inertiaTensor);
        void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
        void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
        void setDamping(const cpfloat linearDamping, const cpfloat angularDamping);
        void setPosition(const Vector3 &position);
        void setPosition(const cpfloat x, const cpfloat y, const cpfloat z);
        void getPosition(Vector3 *position) const;
        Vector3 getPosition() const;
        void setOrientation(const Quaternion &orientation);
        void setOrientation(const cpfloat r, const cpfloat i, const cpfloat j, const cpfloat k);
        void getOrientation(Quaternion *orientation) const;
        void getOrientation(Matrix3 *matrix) const;
        void getOrientation(cpfloat matrix[9]) const;
        Matrix4 getTransform() const;
        void setVelocity(const cpfloat x, const cpfloat y, const cpfloat z);
        Vector3 getVelocity() const;
        void addVelocity(const Vector3 &deltaVelocity);
        void setRotation(const cpfloat x, const cpfloat y, const cpfloat z);
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