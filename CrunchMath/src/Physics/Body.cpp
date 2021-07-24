#include "body.h"
#include <memory.h>
#include <assert.h>

using namespace CrunchPhysx;

static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Quaternion &q,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat)
{
    cpfloat t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    cpfloat t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    cpfloat t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    cpfloat t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    cpfloat t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    cpfloat t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    cpfloat t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    cpfloat t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    cpfloat t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}

static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3 &position,
                                             const Quaternion &orientation)
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
        2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
        2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
        2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
        2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
        2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
        2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
        2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
        2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
        2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.z;
}

void Body::calculateDerivedData()
{
    orientation.normalise();

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
    lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

    // Calculate angular acceleration from torque inputs.
    Vector3 angularAcceleration =
        inverseInertiaTensorWorld.transform(torqueAccum);

    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    velocity.addScaledVector(lastFrameAcceleration, duration);

    // Update angular velocity from both acceleration and impulse.
    rotation.addScaledVector(angularAcceleration, duration);

    // Impose drag.
    velocity *= cp_pow(linearDamping, duration);
    rotation *= cp_pow(angularDamping, duration);

    // Adjust positions
    // Update linear position.
    position.addScaledVector(velocity, duration);

    // Update angular position.
    orientation.addScaledVector(rotation, duration);

    // Normalise the orientation, and update the matrices with the new
    // position and orientation
    calculateDerivedData();

    // Clear accumulators.
    clearAccumulators();

    // Update the kinetic energy store, and possibly put the body to
    // sleep.
    if (canSleep) {
        cpfloat currentMotion = velocity.scalarProduct(velocity) +
            rotation.scalarProduct(rotation);

        cpfloat bias = cp_pow(0.5, duration);
        motion = bias*motion + (1-bias)*currentMotion;

        if (motion < sleepEpsilon) setAwake(false);
        else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
    }
}

void Body::setMass(const cpfloat mass)
{
    assert(mass != 0);
    Body::inverseMass = ((cpfloat)1.0)/mass;
}

cpfloat Body::getMass() const
{
    if (inverseMass == 0) {
        return cp_MAX;
    } else {
        return ((cpfloat)1.0)/inverseMass;
    }
}

cpfloat Body::getInverseMass() const
{
    return inverseMass;
}

void Body::setInertiaTensor(const Matrix3 &inertiaTensor)
{
    inverseInertiaTensor.setInverse(inertiaTensor);
}

void Body::getInertiaTensorWorld(Matrix3 *inertiaTensor) const
{
    inertiaTensor->setInverse(inverseInertiaTensorWorld);
}

void Body::getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const
{
    *inverseInertiaTensor = inverseInertiaTensorWorld;
}

void Body::setDamping(const cpfloat linearDamping,
               const cpfloat angularDamping)
{
    Body::linearDamping = linearDamping;
    Body::angularDamping = angularDamping;
}

void Body::setPosition(const Vector3 &position)
{
    Body::position = position;
}

void Body::setPosition(const cpfloat x, const cpfloat y, const cpfloat z)
{
    position.x = x;
    position.y = y;
    position.z = z;
}

void Body::getPosition(Vector3 *position) const
{
    *position = Body::position;
}

Vector3 Body::getPosition() const
{
    return position;
}

void Body::setOrientation(const Quaternion &orientation)
{
    Body::orientation = orientation;
    Body::orientation.normalise();
}

void Body::setOrientation(const cpfloat r, const cpfloat i,
                   const cpfloat j, const cpfloat k)
{
    orientation.r = r;
    orientation.i = i;
    orientation.j = j;
    orientation.k = k;
    orientation.normalise();
}

void Body::getOrientation(Quaternion *orientation) const
{
    *orientation = Body::orientation;
}

void Body::getOrientation(Matrix3 *matrix) const
{
    getOrientation(matrix->data);
}

void Body::getOrientation(cpfloat matrix[9]) const
{
    matrix[0] = transformMatrix.data[0];
    matrix[1] = transformMatrix.data[1];
    matrix[2] = transformMatrix.data[2];

    matrix[3] = transformMatrix.data[4];
    matrix[4] = transformMatrix.data[5];
    matrix[5] = transformMatrix.data[6];

    matrix[6] = transformMatrix.data[8];
    matrix[7] = transformMatrix.data[9];
    matrix[8] = transformMatrix.data[10];
}

Matrix4 Body::getTransform() const
{
    return transformMatrix;
}

void Body::setVelocity(const cpfloat x, const cpfloat y, const cpfloat z)
{
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

Vector3 Body::getVelocity() const
{
    return velocity;
}

void Body::addVelocity(const Vector3 &deltaVelocity)
{
    velocity += deltaVelocity;
}

void Body::setRotation(const cpfloat x, const cpfloat y, const cpfloat z)
{
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
}

Vector3 Body::getRotation() const
{
    return rotation;
}

void Body::addRotation(const Vector3 &deltaRotation)
{
    rotation += deltaRotation;
}

void Body::setAwake(const bool awake)
{
    if (awake) 
    {
        isAwake= true;

        // Add a bit of motion to avoid it falling asleep immediately.
        motion = sleepEpsilon * 2.0f;
    } 

    else
    {
        isAwake = false;
        velocity.clear();
        rotation.clear();
    }
}

void Body::setCanSleep(const bool canSleep)
{
    Body::canSleep = canSleep;

    if (!canSleep && !isAwake) setAwake();
}

Vector3 Body::getLastFrameAcceleration() const
{
    return lastFrameAcceleration;
}

void Body::clearAccumulators()
{
    forceAccum.clear();
    torqueAccum.clear();
}

void Body::setAcceleration(const Vector3 &acceleration)
{
    Body::acceleration = acceleration;
}