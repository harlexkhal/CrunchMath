#include <memory.h>
#include <assert.h>
#include "Body.h"

namespace CrunchMath
{
    cpfloat sleepEpsilon = ((cpfloat)0.3);

    static inline void _transformInertiaTensor(Mat3x3& iitWorld, const Quaternion& q, const Mat3x3& iitBody, const Mat4x4& rotmat)
    {
        cpfloat t4 = rotmat.Matrix[0][0] * iitBody.Matrix[0][0] +
            rotmat.Matrix[1][0] * iitBody.Matrix[0][1] +
            rotmat.Matrix[2][0] * iitBody.Matrix[0][2];

        cpfloat t9 = rotmat.Matrix[0][0] * iitBody.Matrix[1][0] +
            rotmat.Matrix[1][0] * iitBody.Matrix[1][1] +
            rotmat.Matrix[2][0] * iitBody.Matrix[1][2];

        cpfloat t14 = rotmat.Matrix[0][0] * iitBody.Matrix[2][0] +
            rotmat.Matrix[1][0] * iitBody.Matrix[2][1] +
            rotmat.Matrix[2][0] * iitBody.Matrix[2][2];

        cpfloat t28 = rotmat.Matrix[0][1] * iitBody.Matrix[0][0] +
            rotmat.Matrix[1][1] * iitBody.Matrix[0][1] +
            rotmat.Matrix[2][1] * iitBody.Matrix[0][2];

        cpfloat t33 = rotmat.Matrix[0][1] * iitBody.Matrix[1][0] +
            rotmat.Matrix[1][1] * iitBody.Matrix[1][1] +
            rotmat.Matrix[2][1] * iitBody.Matrix[1][2];

        cpfloat t38 = rotmat.Matrix[0][1] * iitBody.Matrix[2][0] +
            rotmat.Matrix[1][1] * iitBody.Matrix[2][1] +
            rotmat.Matrix[2][1] * iitBody.Matrix[2][2];

        cpfloat t52 = rotmat.Matrix[0][2] * iitBody.Matrix[0][0] +
            rotmat.Matrix[1][2] * iitBody.Matrix[0][1] +
            rotmat.Matrix[2][2] * iitBody.Matrix[0][2];

        cpfloat t57 = rotmat.Matrix[0][2] * iitBody.Matrix[1][0] +
            rotmat.Matrix[1][2] * iitBody.Matrix[1][1] +
            rotmat.Matrix[2][2] * iitBody.Matrix[1][2];

        cpfloat t62 = rotmat.Matrix[0][2] * iitBody.Matrix[2][0] +
            rotmat.Matrix[1][2] * iitBody.Matrix[2][1] +
            rotmat.Matrix[2][2] * iitBody.Matrix[2][2];



        iitWorld.Matrix[0][0] = t4 * rotmat.Matrix[0][0] +
            t9 * rotmat.Matrix[1][0] +
            t14 * rotmat.Matrix[2][0];

        iitWorld.Matrix[1][0] = t4 * rotmat.Matrix[0][1] +
            t9 * rotmat.Matrix[1][1] +
            t14 * rotmat.Matrix[2][1];

        iitWorld.Matrix[2][0] = t4 * rotmat.Matrix[0][2] +
            t9 * rotmat.Matrix[1][2] +
            t14 * rotmat.Matrix[2][2];

        iitWorld.Matrix[0][1] = t28 * rotmat.Matrix[0][0] +
            t33 * rotmat.Matrix[1][0] +
            t38 * rotmat.Matrix[2][0];

        iitWorld.Matrix[1][1] = t28 * rotmat.Matrix[0][1] +
            t33 * rotmat.Matrix[1][1] +
            t38 * rotmat.Matrix[2][1];

        iitWorld.Matrix[2][1] = t28 * rotmat.Matrix[0][2] +
            t33 * rotmat.Matrix[1][2] +
            t38 * rotmat.Matrix[2][2];

        iitWorld.Matrix[0][2] = t52 * rotmat.Matrix[0][0] +
            t57 * rotmat.Matrix[1][0] +
            t62 * rotmat.Matrix[2][0];

        iitWorld.Matrix[1][2] = t52 * rotmat.Matrix[0][1] +
            t57 * rotmat.Matrix[1][1] +
            t62 * rotmat.Matrix[2][1];

        iitWorld.Matrix[2][2] = t52 * rotmat.Matrix[0][2] +
            t57 * rotmat.Matrix[1][2] +
            t62 * rotmat.Matrix[2][2];
    }

    static inline void _calculateTransformMatrix(Mat4x4& transformMatrix, const Vec3& position, const Quaternion& orientation)
    {
        transformMatrix.Rotate(orientation);
        transformMatrix.Translate(position);
    }

    void Body::calculateDerivedData()
    {
        orientation.Normalize();

        // Calculate the transform matrix for the body.
        _calculateTransformMatrix(transformMatrix, position, orientation);

        // Calculate the inertiaTensor in world space.
        _transformInertiaTensor(inverseInertiaTensorWorld,
            orientation,
            inverseInertiaTensor,
            transformMatrix);

    }

    void Body::integrate(cpfloat duration)
    {
        if (!isAwake) return;

        // Calculate linear acceleration from force inputs.
        lastFrameAcceleration = acceleration;
        lastFrameAcceleration += forceAccum * inverseMass;

        // Calculate angular acceleration from torque inputs.
        Vec3 angularAcceleration = inverseInertiaTensorWorld * torqueAccum;

        // Adjust velocities
        // Update linear velocity from both acceleration and impulse.
        velocity += lastFrameAcceleration * duration;

        // Update angular velocity from both acceleration and impulse.
        rotation += angularAcceleration * duration;

        // Impose drag.
        velocity *= cp_pow(linearDamping, duration);
        rotation *= cp_pow(angularDamping, duration);

        // Adjust positions
        // Update linear position.
        position += velocity * duration;

        // Update angular position. //To do=> Take away Quaternion later...
        Quaternion q(0, Vec3(rotation * duration));
        q *= orientation;
        orientation.w += q.w * ((cpfloat)0.5);
        orientation.x += q.x * ((cpfloat)0.5);
        orientation.y += q.y * ((cpfloat)0.5);
        orientation.z += q.z * ((cpfloat)0.5);

        // Normalise the orientation, and update the matrices with the new
        // position and orientation
        calculateDerivedData();

        // Clear accumulators.
        clearAccumulators();

        // Update the kinetic energy store, and possibly put the body to
        // sleep.
        if (canSleep) {
            cpfloat currentMotion = DotProduct(velocity, velocity) + DotProduct(rotation, rotation);

            cpfloat bias = cp_pow(0.5, duration);
            motion = bias * motion + (1 - bias) * currentMotion;

            if (motion < sleepEpsilon) setAwake(false);
            else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
        }
    }

    void Body::setMass(const cpfloat mass)
    {
        assert(mass != 0);
        Body::inverseMass = ((cpfloat)1.0) / mass;
    }

    cpfloat Body::getMass() const
    {
        if (inverseMass == 0) {
            return cp_MAX;
        }
        else {
            return ((cpfloat)1.0) / inverseMass;
        }
    }

    cpfloat Body::getInverseMass() const
    {
        return inverseMass;
    }

    void Body::setInertiaTensor(const Mat3x3& inertiaTensor)
    {
        inverseInertiaTensor = Invert(inertiaTensor);
    }

    void Body::getInertiaTensorWorld(Mat3x3& inertiaTensor) const
    {
        inertiaTensor = Invert(inverseInertiaTensorWorld);
    }

    void Body::getInverseInertiaTensorWorld(Mat3x3& inverseInertiaTensor) const
    {
        inverseInertiaTensor = inverseInertiaTensorWorld;
    }

    void Body::setDamping(const cpfloat linearDamping,
        const cpfloat angularDamping)
    {
        Body::linearDamping = linearDamping;
        Body::angularDamping = angularDamping;
    }

    void Body::setPosition(const Vec3& position)
    {
        Body::position = position;
    }

    void Body::setPosition(const cpfloat x, const cpfloat y, const cpfloat z)
    {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    void Body::getPosition(Vec3* position) const
    {
        *position = Body::position;
    }

    Vec3 Body::getPosition() const
    {
        return position;
    }

    void Body::setOrientation(const Quaternion& orientation)
    {
        Body::orientation = orientation;
        Body::orientation.Normalize();
    }

    void Body::setOrientation(const cpfloat w, const cpfloat x,
        const cpfloat y, const cpfloat z)
    {
        orientation.w = w;
        orientation.x = x;
        orientation.y = y;
        orientation.z = z;
        orientation.Normalize();
    }

    void Body::getOrientation(Quaternion& orientation) const
    {
        orientation = Body::orientation;
    }

    void Body::getOrientation(Mat3x3& matrix) const
    {
        matrix.Matrix[0][0] = transformMatrix.Matrix[0][0];
        matrix.Matrix[0][1] = transformMatrix.Matrix[0][1];
        matrix.Matrix[0][2] = transformMatrix.Matrix[0][2];

        matrix.Matrix[1][0] = transformMatrix.Matrix[1][0];
        matrix.Matrix[1][1] = transformMatrix.Matrix[1][1];
        matrix.Matrix[1][2] = transformMatrix.Matrix[1][2];

        matrix.Matrix[2][0] = transformMatrix.Matrix[2][0];
        matrix.Matrix[2][1] = transformMatrix.Matrix[2][1];
        matrix.Matrix[2][2] = transformMatrix.Matrix[2][2];
    }

    Mat4x4 Body::getTransform() const
    {
        return transformMatrix;
    }

    void Body::setVelocity(const cpfloat x, const cpfloat y, const cpfloat z)
    {
        velocity.x = x;
        velocity.y = y;
        velocity.z = z;
    }

    Vec3 Body::getVelocity() const
    {
        return velocity;
    }

    void Body::addVelocity(const Vec3& deltaVelocity)
    {
        velocity += deltaVelocity;
    }

    void Body::setRotation(const cpfloat x, const cpfloat y, const cpfloat z)
    {
        rotation.x = x;
        rotation.y = y;
        rotation.z = z;
    }

    Vec3 Body::getRotation() const
    {
        return rotation;
    }

    void Body::addRotation(const Vec3& deltaRotation)
    {
        rotation += deltaRotation;
    }

    void Body::setAwake(const bool awake)
    {
        if (awake)
        {
            isAwake = true;

            // Add a bit of motion to avoid it falling asleep immediately.
            motion = sleepEpsilon * 2.0f;
        }

        else
        {
            isAwake = false;
            velocity = Vec3(0.0f, 0.0f, 0.0f);
            rotation = Vec3(0.0f, 0.0f, 0.0f);
        }
    }

    void Body::setCanSleep(const bool canSleep)
    {
        Body::canSleep = canSleep;

        if (!canSleep && !isAwake) setAwake();
    }

    Vec3 Body::getLastFrameAcceleration() const
    {
        return lastFrameAcceleration;
    }

    void Body::clearAccumulators()
    {
        forceAccum = Vec3(0.0f, 0.0f, 0.0f);
        torqueAccum = Vec3(0.0f, 0.0f, 0.0f);
    }

    void Body::setAcceleration(const Vec3& acceleration)
    {
        Body::acceleration = acceleration;
    }

    void Body::setInertiaTensorCoeffs(cpfloat ix, cpfloat iy, cpfloat iz, cpfloat ixy, cpfloat ixz, cpfloat iyz)
    {
        Mat3x3 InertiaTensor;
        InertiaTensor.Matrix[0][0] = ix;   InertiaTensor.Matrix[1][0] = -ixy; InertiaTensor.Matrix[2][0] = -ixz;
        InertiaTensor.Matrix[0][1] = -ixy; InertiaTensor.Matrix[1][1] = iy;   InertiaTensor.Matrix[2][1] = -iyz;
        InertiaTensor.Matrix[0][2] = -ixz; InertiaTensor.Matrix[1][2] = -iyz; InertiaTensor.Matrix[2][2] = iz;

        setInertiaTensor(InertiaTensor);
    }

    void Body::setBlockInertiaTensor(const Vec3& halfSizes, cpfloat mass)
    {
        Vec3 squares = halfSizes * halfSizes;

        setInertiaTensorCoeffs( 0.9f * mass * (squares.y + squares.z),
                                0.9f * mass * (squares.x + squares.z),
                                0.9f * mass * (squares.x + squares.y)
                              );
    }
}