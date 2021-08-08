#include <memory.h>
#include <assert.h>
#include "Contacts.h"

namespace CrunchMath{

    void Contact::setBodyData(Body* one, Body* two, cpfloat friction, cpfloat restitution)
    {
        Contact::body[0] = one;
        Contact::body[1] = two;
        Contact::friction = friction;
        Contact::restitution = restitution;
    }

    void Contact::matchAwakeState()
    {
        // Collisions with the world never cause a body to wake up.
        if (!body[1]) return;

        bool body0awake = body[0]->getAwake();
        bool body1awake = body[1]->getAwake();

        // Wake up only the sleeping one
        if (body0awake ^ body1awake)
        {
            if (body0awake)
                body[1]->setAwake();
            else
                body[0]->setAwake();
        }
    }

    /*
     * Swaps the bodies in the current contact, so body 0 is at body 1 and
     * vice versa. This also changes the direction of the contact normal,
     * but doesn't update any calculated internal data. If you are calling
     * this method manually, then call calculateInternals afterwards to
     * make sure the internal data is up to date.
     */
    void Contact::swapBodies()
    {
        contactNormal *= -1;

        Body* temp = body[0];
        body[0] = body[1];
        body[1] = temp;
    }

    /*
     * Constructs an arbitrary orthonormal basis for the contact.  This is
     * stored as a 3x3 matrix, where each vector is a column (in other
     * words the matrix transforms contact space into world space). The x
     * direction is generated from the contact normal, and the y and z
     * directionss are set so they are at right angles to it.
     */
    inline void Contact::calculateContactBasis()
    {
        Vec3 contactTangent[2];

        // Check whether the Z-axis is nearer to the X or Y axis
        if (cp_abs(contactNormal.x) > cp_abs(contactNormal.y))
        {
            // Scaling factor to ensure the results are normalised
            const cpfloat s = (cpfloat)1.0f / cp_sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

            // The new X-axis is at right angles to the world Y-axis
            contactTangent[0].x = contactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -contactNormal.x * s;

            // The new Y-axis is at right angles to the new X- and Z- axes
            contactTangent[1].x = contactNormal.y * contactTangent[0].x;
            contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
        }

        else
        {
            // Scaling factor to ensure the results are normalised
            const cpfloat s = (cpfloat)1.0 / cp_sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

            // The new X-axis is at right angles to the world X-axis
            contactTangent[0].x = 0;
            contactTangent[0].y = -contactNormal.z * s;
            contactTangent[0].z = contactNormal.y * s;

            // The new Y-axis is at right angles to the new X- and Z- axes
            contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = contactNormal.x * contactTangent[0].y;
        }

        // Make a matrix from the three vectors.
        contactToWorld = Mat3x3(contactNormal, contactTangent[0], contactTangent[1]);
    }

    Vec3 Contact::calculateLocalVelocity(unsigned bodyIndex, cpfloat duration)
    {
        Body* thisBody = body[bodyIndex];

        // Work out the velocity of the contact point.
        Vec3 velocity = CrossProduct(thisBody->getRotation() , relativeContactPosition[bodyIndex]);
        velocity += thisBody->getVelocity();

        // Turn the velocity into contact-coordinates.
        Mat3x3 WorldToContact = contactToWorld;
        WorldToContact.Transpose();
        Vec3 contactVelocity = WorldToContact * velocity;

        // Calculate the ammount of velocity that is due to forces without
        // reactions.
        Vec3 accVelocity = thisBody->getLastFrameAcceleration() * duration;

        // Calculate the velocity in contact-coordinates.
        accVelocity = WorldToContact * accVelocity;

        // We ignore any component of acceleration in the contact normal
        // direction, we are only interested in planar acceleration
        accVelocity.x = 0;

        // Add the planar velocities - if there's enough friction they will
        // be removed during velocity resolution
        contactVelocity += accVelocity;

        // And return it
        return contactVelocity;
    }

    void Contact::calculateDesiredDeltaVelocity(cpfloat duration)
    {
        const static cpfloat velocityLimit = (cpfloat)0.25f;

        // Calculate the acceleration induced velocity accumulated this frame
        cpfloat velocityFromAcc = 0;

        if (body[0]->getAwake())
        {
            velocityFromAcc += DotProduct(body[0]->getLastFrameAcceleration() , contactNormal) * duration;
        }

        if (body[1] && body[1]->getAwake())
        {
            velocityFromAcc -= DotProduct(body[1]->getLastFrameAcceleration(), contactNormal) * duration;
        }

        // If the velocity is very slow, limit the restitution
        cpfloat thisRestitution = restitution;
        if (cp_abs(contactVelocity.x) < velocityLimit)
        {
            thisRestitution = (cpfloat)0.0;
        }

        // Combine the bounce velocity with the removed
        // acceleration velocity.
        desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
    }

    void Contact::calculateInternals(cpfloat duration)
    {
        // Check if the first object is NULL, and swap if it is.
        if (!body[0]) swapBodies();
        assert(body[0]);

        // Calculate an set of axis at the contact point.
        calculateContactBasis();

        // Store the relative position of the contact relative to each body
        relativeContactPosition[0] = contactPoint - body[0]->getPosition();
        if (body[1])
        {
            relativeContactPosition[1] = contactPoint - body[1]->getPosition();
        }

        // Find the relative velocity of the bodies at the contact point.
        contactVelocity = calculateLocalVelocity(0, duration);
        if (body[1])
        {
            contactVelocity -= calculateLocalVelocity(1, duration);
        }

        // Calculate the desired change in velocity for resolution
        calculateDesiredDeltaVelocity(duration);
    }

    void Contact::applyVelocityChange(Vec3 velocityChange[2], Vec3 rotationChange[2])
    {
        // Get hold of the inverse mass and inverse inertia tensor, both in
        // world coordinates.
        Mat3x3 inverseInertiaTensor[2];
        body[0]->getInverseInertiaTensorWorld(inverseInertiaTensor[0]);
        if (body[1])
            body[1]->getInverseInertiaTensorWorld(inverseInertiaTensor[1]);

        // We will calculate the impulse for each contact axis
        Vec3 impulseContact;

        if (friction == (cpfloat)0.0)
        {
            // Use the short format for frictionless Contacts
            impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
        }

        else
        {
            // Otherwise we may have impulses that aren't in the direction of the
            // contact, so we need the more complex version.
            impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
        }

        // Convert impulse to world coordinates
        Vec3 impulse = contactToWorld * impulseContact;

        // Split in the impulse into linear and rotational components
        Vec3 impulsiveTorque = CrossProduct(relativeContactPosition[0] , impulse);
        rotationChange[0] = inverseInertiaTensor[0] * impulsiveTorque;
        velocityChange[0] = Vec3(0.0f, 0.0f, 0.0f);
        velocityChange[0] += impulse * body[0]->getInverseMass();

        // Apply the changes
        body[0]->addVelocity(velocityChange[0]);
        body[0]->addRotation(rotationChange[0]);

        if (body[1])
        {
            // Work out body one's linear and angular changes
            Vec3 impulsiveTorque = CrossProduct(impulse , relativeContactPosition[1]);
            rotationChange[1] = inverseInertiaTensor[1] * impulsiveTorque;
            velocityChange[1] = Vec3(0.0f, 0.0f, 0.0f);
            velocityChange[1] += impulse * -body[1]->getInverseMass();

            // And apply them.
            body[1]->addVelocity(velocityChange[1]);
            body[1]->addRotation(rotationChange[1]);
        }
    }

    inline Vec3 Contact::calculateFrictionlessImpulse(Mat3x3* inverseInertiaTensor)
    {
        Vec3 impulseContact;

        // Build a vector that shows the change in velocity in
        // world space for a unit impulse in the direction of the contact
        // normal.
        Vec3 deltaVelWorld = CrossProduct(relativeContactPosition[0] , contactNormal);
        deltaVelWorld = inverseInertiaTensor[0] * deltaVelWorld;
        deltaVelWorld = CrossProduct(deltaVelWorld , relativeContactPosition[0]);

        // Work out the change in velocity in contact coordiantes.
        cpfloat deltaVelocity = DotProduct(deltaVelWorld , contactNormal);

        // Add the linear component of velocity change
        deltaVelocity += body[0]->getInverseMass();

        // Check if we need to the second body's data
        if (body[1])
        {
            // Go through the same transformation sequence again
            Vec3 deltaVelWorld = CrossProduct(relativeContactPosition[1] , contactNormal);
            deltaVelWorld = inverseInertiaTensor[1] * deltaVelWorld;
            deltaVelWorld = CrossProduct(deltaVelWorld , relativeContactPosition[1]);

            // Add the change in velocity due to rotation
            deltaVelocity += DotProduct(deltaVelWorld, contactNormal);

            // Add the change in velocity due to linear motion
            deltaVelocity += body[1]->getInverseMass();
        }

        // Calculate the required size of the impulse
        impulseContact.x = desiredDeltaVelocity / deltaVelocity;
        impulseContact.y = 0;
        impulseContact.z = 0;
        return impulseContact;
    }

    inline Vec3 Contact::calculateFrictionImpulse(Mat3x3* inverseInertiaTensor)
    {
        Vec3 impulseContact;
        cpfloat inverseMass = body[0]->getInverseMass();

        // The equivalent of a cross product in matrices is multiplication
        // by a skew symmetric matrix - we build the matrix for converting
        // between linear and angular quantities.
        Mat3x3 impulseToTorque;
        impulseToTorque.SetSkewSymmetric(relativeContactPosition[0]);

        // Build the matrix to convert contact impulse to change in velocity
        // in world coordinates.
        Mat3x3 deltaVelWorld = impulseToTorque;
        deltaVelWorld *= inverseInertiaTensor[0];
        deltaVelWorld *= impulseToTorque;
        deltaVelWorld *= -1;

        // Check if we need to add body two's data
        if (body[1])
        {
            // Set the cross product matrix
            impulseToTorque.SetSkewSymmetric(relativeContactPosition[1]);

            // Calculate the velocity change matrix
            Mat3x3 deltaVelWorld2 = impulseToTorque;
            deltaVelWorld2 *= inverseInertiaTensor[1];
            deltaVelWorld2 *= impulseToTorque;
            deltaVelWorld2 *= -1;

            // Add to the total delta velocity.
            deltaVelWorld += deltaVelWorld2;

            // Add to the inverse mass
            inverseMass += body[1]->getInverseMass();
        }

        // Do a change of basis to convert into contact coordinates.
        Mat3x3 WorldToContact = contactToWorld;
        WorldToContact.Transpose();

        Mat3x3 deltaVelocity = WorldToContact;
        deltaVelocity *= deltaVelWorld;
        deltaVelocity *= contactToWorld;

        // Add in the linear velocity change
        deltaVelocity.Matrix[0][0] += inverseMass;
        deltaVelocity.Matrix[1][1] += inverseMass;
        deltaVelocity.Matrix[2][2] += inverseMass;

        // Invert to get the impulse needed per unit velocity
        Mat3x3 impulseMatrix = Invert(deltaVelocity);

        // Find the target velocities to kill
        Vec3 velKill(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);

        // Find the impulse to kill target velocities
        impulseContact = impulseMatrix * velKill;

        // Check for exceeding friction
        cpfloat planarImpulse = cp_sqrt(
            impulseContact.y * impulseContact.y +
            impulseContact.z * impulseContact.z
        );

        if (planarImpulse > impulseContact.x * friction)
        {
            // We need to use dynamic friction
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = deltaVelocity.Matrix[0][0] + deltaVelocity.Matrix[1][0] * friction * impulseContact.y + deltaVelocity.Matrix[2][0] * friction * impulseContact.z;
            impulseContact.x = desiredDeltaVelocity / impulseContact.x;
            impulseContact.y *= friction * impulseContact.x;
            impulseContact.z *= friction * impulseContact.x;
        }
        return impulseContact;
    }

    void Contact::applyPositionChange(Vec3 linearChange[2], Vec3 angularChange[2], cpfloat penetration)
    {
        const cpfloat angularLimit = (cpfloat)0.2f;
        cpfloat angularMove[2];
        cpfloat linearMove[2];

        cpfloat totalInertia = 0;
        cpfloat linearInertia[2];
        cpfloat angularInertia[2];

        // We need to work out the inertia of each object in the direction
        // of the contact normal, due to angular inertia only.
        for (unsigned i = 0; i < 2; i++) if (body[i])
        {
            Mat3x3 inverseInertiaTensor;
            body[i]->getInverseInertiaTensorWorld(inverseInertiaTensor);

            // Use the same procedure as for calculating frictionless
            // velocity change to work out the angular inertia.
            Vec3 angularInertiaWorld = CrossProduct(relativeContactPosition[i] , contactNormal);
            angularInertiaWorld = inverseInertiaTensor * angularInertiaWorld;
            angularInertiaWorld = CrossProduct(angularInertiaWorld , relativeContactPosition[i]);
            angularInertia[i] = DotProduct(angularInertiaWorld , contactNormal);

            // The linear component is simply the inverse mass
            linearInertia[i] = body[i]->getInverseMass();

            // Keep track of the total inertia from all components
            totalInertia += linearInertia[i] + angularInertia[i];

            // We break the loop here so that the totalInertia value is
            // completely calculated (by both iterations) before
            // continuing.
        }

        // Loop through again calculating and applying the changes
        for (unsigned i = 0; i < 2; i++)
        {
            if (body[i])
            {
                // The linear and angular movements required are in proportion to
                // the two inverse inertias.
                cpfloat sign = (i == 0) ? 1 : -1;
                angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
                linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

                // To avoid angular projections that are too great (when mass is large
                // but inertia tensor is small) limit the angular move.
                Vec3 projection = relativeContactPosition[i];
                projection += contactNormal * DotProduct(-relativeContactPosition[i] , contactNormal);

                // Use the small angle approximation for the sine of the angle (i.e.
                // the magnitude would be sine(angularLimit) * projection.magnitude
                // but we approximate sine(angularLimit) to angularLimit).
                cpfloat maxMagnitude = angularLimit * cp_sqrt(DotProduct(projection, projection));

                if (angularMove[i] < -maxMagnitude)
                {
                    cpfloat totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = -maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }

                else if (angularMove[i] > maxMagnitude)
                {
                    cpfloat totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }

                // We have the linear amount of movement required by turning
                // the rigid body (in angularMove[i]). We now need to
                // calculate the desired rotation to achieve that.
                if (angularMove[i] == 0)
                {
                    // Easy case - no angular movement means no rotation.
                    angularChange[i] = Vec3(0.0f, 0.0f, 0.0f);
                }

                else
                {
                    // Work out the direction we'd like to rotate in.
                    Vec3 targetAngularDirection = CrossProduct(relativeContactPosition[i], contactNormal);

                    Mat3x3 inverseInertiaTensor;
                    body[i]->getInverseInertiaTensorWorld(inverseInertiaTensor);

                    // Work out the direction we'd need to rotate to achieve that
                    angularChange[i] = inverseInertiaTensor * targetAngularDirection * (angularMove[i] / angularInertia[i]);
                }

                // Velocity change is easier - it is just the linear movement
                // along the contact normal.
                linearChange[i] = contactNormal * linearMove[i];

                // Now we can start to apply the values we've calculated.
                // Apply the linear movement
                Vec3 pos;
                body[i]->getPosition(&pos);
                pos += contactNormal * linearMove[i];
                body[i]->setPosition(pos);

                // And the change in orientation
                Quaternion q;
                body[i]->getOrientation(q);

                Quaternion Rq(0, Vec3(angularChange[i] * 1.0f));
                Rq *= q;
                q.w += Rq.w * ((cpfloat)0.5);
                q.x += Rq.x * ((cpfloat)0.5);
                q.y += Rq.y * ((cpfloat)0.5);
                q.z += Rq.z * ((cpfloat)0.5);

                body[i]->setOrientation(q);

                // We need to calculate the derived data for any body that is
                // asleep, so that the changes are reflected in the object's
                // data. Otherwise the resolution will not change the position
                // of the object, and the next collision detection round will
                // have the same penetration.
                if (!body[i]->getAwake()) body[i]->calculateDerivedData();
            }
        }
    }

    // Contact resolver implementation
    ContactResolver::ContactResolver(unsigned iterations, cpfloat velocityEpsilon, cpfloat positionEpsilon)
    {
        setIterations(iterations, iterations);
        setEpsilon(velocityEpsilon, positionEpsilon);
    }

    void ContactResolver::setIterations(unsigned velocityIterations, unsigned positionIterations)
    {
        ContactResolver::velocityIterations = velocityIterations;
        ContactResolver::positionIterations = positionIterations;
    }

    void ContactResolver::setEpsilon(cpfloat velocityEpsilon, cpfloat positionEpsilon)
    {
        ContactResolver::velocityEpsilon = velocityEpsilon;
        ContactResolver::positionEpsilon = positionEpsilon;
    }

    void ContactResolver::resolveContacts(Contact* Contacts, unsigned numContacts, cpfloat duration)
    {
        // Make sure we have something to do.
        if (numContacts == 0) return;
        if (!isValid()) return;

        // Prepare the Contacts for processing
        prepareContacts(Contacts, numContacts, duration);

        // Resolve the interpenetration problems with the Contacts.
        adjustPositions(Contacts, numContacts, duration);

        // Resolve the velocity problems with the Contacts.
        adjustVelocities(Contacts, numContacts, duration);
    }

    void ContactResolver::prepareContacts(Contact* Contacts, unsigned numContacts, cpfloat duration)
    {
        // Generate contact velocity and axis information.
        Contact* lastContact = Contacts + numContacts;
        for (Contact* contact = Contacts; contact < lastContact; contact++)
        {
            // Calculate the internal contact data (inertia, basis, etc).
            contact->calculateInternals(duration);
        }
    }

    void ContactResolver::adjustVelocities(Contact* c, unsigned numContacts, cpfloat duration)
    {
        Vec3 velocityChange[2], rotationChange[2];
        Vec3 deltaVel;

        // iteratively handle impacts in order of severity.
        velocityIterationsUsed = 0;
        while (velocityIterationsUsed < velocityIterations)
        {
            // Find contact with maximum magnitude of probable velocity change.
            cpfloat max = velocityEpsilon;
            unsigned index = numContacts;
            for (unsigned i = 0; i < numContacts; i++)
            {
                if (c[i].desiredDeltaVelocity > max)
                {
                    max = c[i].desiredDeltaVelocity;
                    index = i;
                }
            }

            if (index == numContacts)
                break;

            // Match the awake state at the contact
            c[index].matchAwakeState();

            // Do the resolution on the contact that came out top.
            c[index].applyVelocityChange(velocityChange, rotationChange);

            // With the change in velocity of the two bodies, the update of
            // contact velocities means that some of the relative closing
            // velocities need recomputing.
            for (unsigned i = 0; i < numContacts; i++)
            {
                // Check each body in the contact
                for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
                {
                    // Check for a match with each body in the newly
                    // resolved contact
                    for (unsigned d = 0; d < 2; d++)
                    {
                        if (c[i].body[b] == c[index].body[d])
                        {
                            deltaVel = velocityChange[d] + CrossProduct(rotationChange[d], c[i].relativeContactPosition[b]);

                            // The sign of the change is negative if we're dealing
                            // with the second body in a contact.
                            Mat3x3 WorldToContact = c[i].contactToWorld;
                            WorldToContact.Transpose();

                            c[i].contactVelocity += (WorldToContact * deltaVel) * (b ? -1 : 1);
                            c[i].calculateDesiredDeltaVelocity(duration);
                        }
                    }
                }
            }
            velocityIterationsUsed++;
        }
    }

    void ContactResolver::adjustPositions(Contact* c, unsigned numContacts, cpfloat duration)
    {
        unsigned i, index;
        Vec3 linearChange[2], angularChange[2];
        cpfloat max;
        Vec3 deltaPosition;

        // iteratively resolve interpenetrations in order of severity.
        positionIterationsUsed = 0;
        while (positionIterationsUsed < positionIterations)
        {
            // Find biggest penetration
            max = positionEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++)
            {
                if (c[i].penetration > max)
                {
                    max = c[i].penetration;
                    index = i;
                }
            }

            if (index == numContacts)
                break;

            // Match the awake state at the contact
            c[index].matchAwakeState();

            // Resolve the penetration.
            c[index].applyPositionChange(linearChange, angularChange, max);

            // Again this action may have changed the penetration of other
            // bodies, so we update Contacts.
            for (i = 0; i < numContacts; i++)
            {
                // Check each body in the contact
                for (unsigned b = 0; b < 2; b++)
                {
                    if (c[i].body[b])
                    {
                        // Check for a match with each body in the newly
                        // resolved contact
                        for (unsigned d = 0; d < 2; d++)
                        {
                            if (c[i].body[b] == c[index].body[d])
                            {
                                deltaPosition = linearChange[d] + CrossProduct(angularChange[d], c[i].relativeContactPosition[b]);

                                // The sign of the change is positive if we're
                                // dealing with the second body in a contact
                                // and negative otherwise (because we're
                                // subtracting the resolution)..
                                c[i].penetration += DotProduct(deltaPosition, c[i].contactNormal) * (b ? 1 : -1);
                            }
                        }
                    }
                }
            }
            positionIterationsUsed++;
        }
    }
}