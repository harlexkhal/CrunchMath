#include <memory.h>
#include <assert.h>
#include "Contacts.h"

namespace CrunchMath{

    void Contact::setBodyData(Body* one, Body* two, float Friction, float Restitution)
    {
        Contact::body[0] = one;
        Contact::body[1] = two;
        Contact::Friction = Friction;
        Contact::Restitution = Restitution;
    }

    void Contact::MatchAwakeState()
    {
        // Collisions with the world never cause a body to wake up.
        if (!body[1]) return;

        bool body0awake = body[0]->GetAwake();
        bool body1awake = body[1]->GetAwake();

        // Wake up only the sleeping one
        if (body0awake ^ body1awake)
        {
            if (body0awake)
                body[1]->SetAwake();
            else
                body[0]->SetAwake();
        }
    }

    /*
     * Swaps the bodies in the current contact, so body 0 is at body 1 and
     * vice versa. This also changes the direction of the contact normal,
     * but doesn't update any Calculated internal data. If you are calling
     * this method manually, then call CalculateInternals afterwards to
     * make sure the internal data is up to date.
     */
    void Contact::SwapBodies()
    {
        ContactNormal *= -1;

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
    inline void Contact::CalculateContactBasis()
    {
        Vec3 contactTangent[2];

        // Check whether the Z-axis is nearer to the X or Y axis
        if (fabsf(ContactNormal.x) > fabsf(ContactNormal.y))
        {
            // Scaling factor to ensure the results are normalised
            const float s = (float)1.0f / sqrtf(ContactNormal.z * ContactNormal.z + ContactNormal.x * ContactNormal.x);

            // The new X-axis is at right angles to the world Y-axis
            contactTangent[0].x = ContactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -ContactNormal.x * s;

            // The new Y-axis is at right angles to the new X- and Z- axes
            contactTangent[1].x = ContactNormal.y * contactTangent[0].x;
            contactTangent[1].y = ContactNormal.z * contactTangent[0].x - ContactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -ContactNormal.y * contactTangent[0].x;
        }

        else
        {
            // Scaling factor to ensure the results are normalised
            const float s = (float)1.0 / sqrtf(ContactNormal.z * ContactNormal.z + ContactNormal.y * ContactNormal.y);

            // The new X-axis is at right angles to the world X-axis
            contactTangent[0].x = 0;
            contactTangent[0].y = -ContactNormal.z * s;
            contactTangent[0].z = ContactNormal.y * s;

            // The new Y-axis is at right angles to the new X- and Z- axes
            contactTangent[1].x = ContactNormal.y * contactTangent[0].z - ContactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -ContactNormal.x * contactTangent[0].z;
            contactTangent[1].z = ContactNormal.x * contactTangent[0].y;
        }

        // Make a matrix from the three vectors.
        ContactToWorld = Mat3x3(ContactNormal, contactTangent[0], contactTangent[1]);
    }

    Vec3 Contact::CalculateLocalVelocity(unsigned bodyIndex, float duration)
    {
        Body* thisBody = body[bodyIndex];

        // Work out the Velocity of the contact point.
        Vec3 Velocity = CrossProduct(thisBody->GetRotation() , RelativeContactPosition[bodyIndex]);
        Velocity += thisBody->GetVelocity();

        // Turn the Velocity into contact-coordinates.
        Mat3x3 WorldToContact = ContactToWorld;
        WorldToContact.Transpose();
        Vec3 ContactVelocity = WorldToContact * Velocity;

        // Calculate the ammount of Velocity that is due to forces without
        // reactions.
        Vec3 accVelocity = thisBody->GetLastFrameAcceleration() * duration;

        // Calculate the Velocity in contact-coordinates.
        accVelocity = WorldToContact * accVelocity;

        // We ignore any component of Acceleration in the contact normal
        // direction, we are only interested in planar Acceleration
        accVelocity.x = 0;

        // Add the planar velocities - if there's enough Friction they will
        // be removed during Velocity resolution
        ContactVelocity += accVelocity;

        // And return it
        return ContactVelocity;
    }

    void Contact::CalculateDesiredDeltaVelocity(float duration)
    {
        const static float VelocityLimit = (float)0.25f;

        // Calculate the Acceleration induced Velocity accumulated this frame
        float VelocityFromAcc = 0;

        if (body[0]->GetAwake())
        {
            VelocityFromAcc += DotProduct(body[0]->GetLastFrameAcceleration() , ContactNormal) * duration;
        }

        if (body[1] && body[1]->GetAwake())
        {
            VelocityFromAcc -= DotProduct(body[1]->GetLastFrameAcceleration(), ContactNormal) * duration;
        }

        // If the Velocity is very slow, limit the Restitution
        float thisRestitution = Restitution;
        if (fabsf(ContactVelocity.x) < VelocityLimit)
        {
            thisRestitution = (float)0.0;
        }

        // Combine the bounce Velocity with the removed
        // Acceleration Velocity.
        DesiredDeltaVelocity = -ContactVelocity.x - thisRestitution * (ContactVelocity.x - VelocityFromAcc);
    }

    void Contact::CalculateInternals(float duration)
    {
        // Check if the first object is NULL, and swap if it is.
        if (!body[0]) SwapBodies();
        assert(body[0]);

        // Calculate an set of axis at the contact point.
        CalculateContactBasis();

        // Store the relative Position of the contact relative to each body
        RelativeContactPosition[0] = ContactPoint - body[0]->GetPosition();
        if (body[1])
        {
            RelativeContactPosition[1] = ContactPoint - body[1]->GetPosition();
        }

        // Find the relative Velocity of the bodies at the contact point.
        ContactVelocity = CalculateLocalVelocity(0, duration);
        if (body[1])
        {
            ContactVelocity -= CalculateLocalVelocity(1, duration);
        }

        // Calculate the desired change in Velocity for resolution
        CalculateDesiredDeltaVelocity(duration);
    }

    void Contact::ApplyVelocityChange(Vec3 VelocityChange[2], Vec3 RotationChange[2])
    {
        // Get hold of the inverse mass and inverse inertia tensor, both in
        // world coordinates.
        Mat3x3 InverseInertiaTensor[2];
        body[0]->GetInverseInertiaTensorWorld(InverseInertiaTensor[0]);
        if (body[1])
            body[1]->GetInverseInertiaTensorWorld(InverseInertiaTensor[1]);

        // We will Calculate the impulse for each contact axis
        Vec3 impulseContact;

        if (Friction == (float)0.0)
        {
            // Use the short format for Frictionless Contacts
            impulseContact = CalculateFrictionlessImpulse(InverseInertiaTensor);
        }

        else
        {
            // Otherwise we may have impulses that aren't in the direction of the
            // contact, so we need the more complex version.
            impulseContact = CalculateFrictionImpulse(InverseInertiaTensor);
        }

        // Convert impulse to world coordinates
        Vec3 impulse = ContactToWorld * impulseContact;

        // Split in the impulse into linear and Rotational components
        Vec3 impulsiveTorque = CrossProduct(RelativeContactPosition[0] , impulse);
        RotationChange[0] = InverseInertiaTensor[0] * impulsiveTorque;
        VelocityChange[0] = Vec3(0.0f, 0.0f, 0.0f);
        VelocityChange[0] += impulse * body[0]->GetInverseMass();

        // Apply the changes
        body[0]->AddVelocity(VelocityChange[0]);
        body[0]->AddRotation(RotationChange[0]);

        if (body[1])
        {
            // Work out body one's linear and angular changes
            Vec3 impulsiveTorque = CrossProduct(impulse , RelativeContactPosition[1]);
            RotationChange[1] = InverseInertiaTensor[1] * impulsiveTorque;
            VelocityChange[1] = Vec3(0.0f, 0.0f, 0.0f);
            VelocityChange[1] += impulse * -body[1]->GetInverseMass();

            // And Apply them.
            body[1]->AddVelocity(VelocityChange[1]);
            body[1]->AddRotation(RotationChange[1]);
        }
    }

    inline Vec3 Contact::CalculateFrictionlessImpulse(Mat3x3* InverseInertiaTensor)
    {
        Vec3 impulseContact;

        // Build a vector that shows the change in Velocity in
        // world space for a unit impulse in the direction of the contact
        // normal.
        Vec3 deltaVelWorld = CrossProduct(RelativeContactPosition[0] , ContactNormal);
        deltaVelWorld = InverseInertiaTensor[0] * deltaVelWorld;
        deltaVelWorld = CrossProduct(deltaVelWorld , RelativeContactPosition[0]);

        // Work out the change in Velocity in contact coordiantes.
        float deltaVelocity = DotProduct(deltaVelWorld , ContactNormal);

        // Add the linear component of Velocity change
        deltaVelocity += body[0]->GetInverseMass();

        // Check if we need to the second body's data
        if (body[1])
        {
            // Go through the same transformation sequence again
            Vec3 deltaVelWorld = CrossProduct(RelativeContactPosition[1] , ContactNormal);
            deltaVelWorld = InverseInertiaTensor[1] * deltaVelWorld;
            deltaVelWorld = CrossProduct(deltaVelWorld , RelativeContactPosition[1]);

            // Add the change in Velocity due to Rotation
            deltaVelocity += DotProduct(deltaVelWorld, ContactNormal);

            // Add the change in Velocity due to linear Motion
            deltaVelocity += body[1]->GetInverseMass();
        }

        // Calculate the required size of the impulse
        impulseContact.x = DesiredDeltaVelocity / deltaVelocity;
        impulseContact.y = 0;
        impulseContact.z = 0;
        return impulseContact;
    }

    inline Vec3 Contact::CalculateFrictionImpulse(Mat3x3* InverseInertiaTensor)
    {
        Vec3 impulseContact;
        float InverseMass = body[0]->GetInverseMass();

        // The equivalent of a cross product in matrices is multiplication
        // by a skew symmetric matrix - we build the matrix for converting
        // between linear and angular quantities.
        Mat3x3 impulseToTorque;
        impulseToTorque.SetSkewSymmetric(RelativeContactPosition[0]);

        // Build the matrix to convert contact impulse to change in Velocity
        // in world coordinates.
        Mat3x3 deltaVelWorld = impulseToTorque;
        deltaVelWorld *= InverseInertiaTensor[0];
        deltaVelWorld *= impulseToTorque;
        deltaVelWorld *= -1;

        // Check if we need to add body two's data
        if (body[1])
        {
            // Set the cross product matrix
            impulseToTorque.SetSkewSymmetric(RelativeContactPosition[1]);

            // Calculate the Velocity change matrix
            Mat3x3 deltaVelWorld2 = impulseToTorque;
            deltaVelWorld2 *= InverseInertiaTensor[1];
            deltaVelWorld2 *= impulseToTorque;
            deltaVelWorld2 *= -1;

            // Add to the total delta Velocity.
            deltaVelWorld += deltaVelWorld2;

            // Add to the inverse mass
            InverseMass += body[1]->GetInverseMass();
        }

        // Do a change of basis to convert into contact coordinates.
        Mat3x3 WorldToContact = ContactToWorld;
        WorldToContact.Transpose();

        Mat3x3 deltaVelocity = WorldToContact;
        deltaVelocity *= deltaVelWorld;
        deltaVelocity *= ContactToWorld;

        // Add in the linear Velocity change
        deltaVelocity.Matrix[0][0] += InverseMass;
        deltaVelocity.Matrix[1][1] += InverseMass;
        deltaVelocity.Matrix[2][2] += InverseMass;

        // Invert to get the impulse needed per unit Velocity
        Mat3x3 impulseMatrix = Invert(deltaVelocity);

        // Find the target velocities to kill
        Vec3 velKill(DesiredDeltaVelocity, -ContactVelocity.y, -ContactVelocity.z);

        // Find the impulse to kill target velocities
        impulseContact = impulseMatrix * velKill;

        // Check for exceeding Friction
        float planarImpulse = sqrtf(
            impulseContact.y * impulseContact.y +
            impulseContact.z * impulseContact.z
        );

        if (planarImpulse > impulseContact.x * Friction)
        {
            // We need to use dynamic Friction
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = deltaVelocity.Matrix[0][0] + deltaVelocity.Matrix[1][0] * Friction * impulseContact.y + deltaVelocity.Matrix[2][0] * Friction * impulseContact.z;
            impulseContact.x = DesiredDeltaVelocity / impulseContact.x;
            impulseContact.y *= Friction * impulseContact.x;
            impulseContact.z *= Friction * impulseContact.x;
        }
        return impulseContact;
    }

    void Contact::ApplyPositionChange(Vec3 linearChange[2], Vec3 angularChange[2], float Penetration)
    {
        const float angularLimit = (float)0.2f;
        float angularMove[2];
        float linearMove[2];

        float totalInertia = 0;
        float linearInertia[2];
        float angularInertia[2];

        // We need to work out the inertia of each object in the direction
        // of the contact normal, due to angular inertia only.
        for (unsigned i = 0; i < 2; i++) if (body[i])
        {
            Mat3x3 InverseInertiaTensor;
            body[i]->GetInverseInertiaTensorWorld(InverseInertiaTensor);

            // Use the same procedure as for calculating Frictionless
            // Velocity change to work out the angular inertia.
            Vec3 angularInertiaWorld = CrossProduct(RelativeContactPosition[i] , ContactNormal);
            angularInertiaWorld = InverseInertiaTensor * angularInertiaWorld;
            angularInertiaWorld = CrossProduct(angularInertiaWorld , RelativeContactPosition[i]);
            angularInertia[i] = DotProduct(angularInertiaWorld , ContactNormal);

            // The linear component is simply the inverse mass
            linearInertia[i] = body[i]->GetInverseMass();

            // Keep track of the total inertia from all components
            totalInertia += linearInertia[i] + angularInertia[i];

            // We break the loop here so that the totalInertia value is
            // completely Calculated (by both iterations) before
            // continuing.
        }

        // Loop through again calculating and Applying the changes
        for (unsigned i = 0; i < 2; i++)
        {
            if (body[i])
            {
                // The linear and angular movements required are in proportion to
                // the two inverse inertias.
                float sign = (i == 0) ? 1 : -1;
                angularMove[i] = sign * Penetration * (angularInertia[i] / totalInertia);
                linearMove[i] = sign * Penetration * (linearInertia[i] / totalInertia);

                // To avoid angular projections that are too great (when mass is large
                // but inertia tensor is small) limit the angular move.
                Vec3 projection = RelativeContactPosition[i];
                projection += ContactNormal * DotProduct(-RelativeContactPosition[i] , ContactNormal);

                // Use the small angle approximation for the sine of the angle (i.e.
                // the magnitude would be sine(angularLimit) * projection.magnitude
                // but we approximate sine(angularLimit) to angularLimit).
                float maxMagnitude = angularLimit * sqrtf(DotProduct(projection, projection));

                if (angularMove[i] < -maxMagnitude)
                {
                    float totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = -maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }

                else if (angularMove[i] > maxMagnitude)
                {
                    float totalMove = angularMove[i] + linearMove[i];
                    angularMove[i] = maxMagnitude;
                    linearMove[i] = totalMove - angularMove[i];
                }

                // We have the linear amount of movement required by turning
                // the rigid body (in angularMove[i]). We now need to
                // Calculate the desired Rotation to achieve that.
                if (angularMove[i] == 0)
                {
                    // Easy case - no angular movement means no Rotation.
                    angularChange[i] = Vec3(0.0f, 0.0f, 0.0f);
                }

                else
                {
                    // Work out the direction we'd like to rotate in.
                    Vec3 targetAngularDirection = CrossProduct(RelativeContactPosition[i], ContactNormal);

                    Mat3x3 InverseInertiaTensor;
                    body[i]->GetInverseInertiaTensorWorld(InverseInertiaTensor);

                    // Work out the direction we'd need to rotate to achieve that
                    angularChange[i] = InverseInertiaTensor * targetAngularDirection * (angularMove[i] / angularInertia[i]);
                }

                // Velocity change is easier - it is just the linear movement
                // along the contact normal.
                linearChange[i] = ContactNormal * linearMove[i];

                // Now we can start to Apply the values we've Calculated.
                // Apply the linear movement
                Vec3 pos;
                body[i]->GetPosition(pos);
                pos += ContactNormal * linearMove[i];
                body[i]->SetPosition(pos);

                // And the change in Orientation
                Quaternion q;
                body[i]->GetOrientation(q);

                Quaternion Rq(0, Vec3(angularChange[i] * 1.0f));
                Rq *= q;
                q.w += Rq.w * ((float)0.5);
                q.x += Rq.x * ((float)0.5);
                q.y += Rq.y * ((float)0.5);
                q.z += Rq.z * ((float)0.5);

                body[i]->SetOrientation(q);

                // We need to Calculate the derived data for any body that is
                // asleep, so that the changes are reflected in the object's
                // data. Otherwise the resolution will not change the Position
                // of the object, and the next collision detection round will
                // have the same Penetration.
                if (!body[i]->GetAwake()) body[i]->CalculateDerivedData();
            }
        }
    }

    // Contact Resolver implementation
    ContactResolver::ContactResolver(unsigned iterations, float VelocityEpsilon, float PositionEpsilon)
    {
        SetIterations(iterations, iterations);
        SetEpsilon(VelocityEpsilon, PositionEpsilon);
    }

    void ContactResolver::SetIterations(unsigned VelocityIterations, unsigned PositionIterations)
    {
        ContactResolver::VelocityIterations = VelocityIterations;
        ContactResolver::PositionIterations = PositionIterations;
    }

    void ContactResolver::SetEpsilon(float VelocityEpsilon, float PositionEpsilon)
    {
        ContactResolver::VelocityEpsilon = VelocityEpsilon;
        ContactResolver::PositionEpsilon = PositionEpsilon;
    }

    void ContactResolver::ResolveContacts(Contact* Contacts, unsigned numContacts, float duration)
    {
        // Make sure we have something to do.
        if (numContacts == 0) return;
        if (!IsValid()) return;

        // Prepare the Contacts for processing
        PrepareContacts(Contacts, numContacts, duration);

        // Resolve the interPenetration problems with the Contacts.
        AdjustPositions(Contacts, numContacts, duration);

        // Resolve the Velocity problems with the Contacts.
        AdjustVelocities(Contacts, numContacts, duration);
    }

    void ContactResolver::PrepareContacts(Contact* Contacts, unsigned numContacts, float duration)
    {
        // Generate contact Velocity and axis information.
        Contact* lastContact = Contacts + numContacts;
        for (Contact* contact = Contacts; contact < lastContact; contact++)
        {
            // Calculate the internal contact data (inertia, basis, etc).
            contact->CalculateInternals(duration);
        }
    }

    void ContactResolver::AdjustVelocities(Contact* c, unsigned numContacts, float duration)
    {
        Vec3 VelocityChange[2], RotationChange[2];
        Vec3 deltaVel;

        // iteratively handle impacts in order of severity.
        VelocityIterationsUsed = 0;
        while (VelocityIterationsUsed < VelocityIterations)
        {
            // Find contact with maximum magnitude of probable Velocity change.
            float max = VelocityEpsilon;
            unsigned index = numContacts;
            for (unsigned i = 0; i < numContacts; i++)
            {
                if (c[i].DesiredDeltaVelocity > max)
                {
                    max = c[i].DesiredDeltaVelocity;
                    index = i;
                }
            }

            if (index == numContacts)
                break;

            // Match the awake state at the contact
            c[index].MatchAwakeState();

            // Do the resolution on the contact that came out top.
            c[index].ApplyVelocityChange(VelocityChange, RotationChange);

            // With the change in Velocity of the two bodies, the update of
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
                            deltaVel = VelocityChange[d] + CrossProduct(RotationChange[d], c[i].RelativeContactPosition[b]);

                            // The sign of the change is negative if we're dealing
                            // with the second body in a contact.
                            Mat3x3 WorldToContact = c[i].ContactToWorld;
                            WorldToContact.Transpose();

                            c[i].ContactVelocity += (WorldToContact * deltaVel) * (b ? -1 : 1);
                            c[i].CalculateDesiredDeltaVelocity(duration);
                        }
                    }
                }
            }
            VelocityIterationsUsed++;
        }
    }

    void ContactResolver::AdjustPositions(Contact* c, unsigned numContacts, float duration)
    {
        unsigned i, index;
        Vec3 linearChange[2], angularChange[2];
        float max;
        Vec3 deltaPosition;

        // iteratively resolve interPenetrations in order of severity.
        PositionIterationsUsed = 0;
        while (PositionIterationsUsed < PositionIterations)
        {
            // Find biggest Penetration
            max = PositionEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++)
            {
                if (c[i].Penetration > max)
                {
                    max = c[i].Penetration;
                    index = i;
                }
            }

            if (index == numContacts)
                break;

            // Match the awake state at the contact
            c[index].MatchAwakeState();

            // Resolve the Penetration.
            c[index].ApplyPositionChange(linearChange, angularChange, max);

            // Again this action may have changed the Penetration of other
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
                                deltaPosition = linearChange[d] + CrossProduct(angularChange[d], c[i].RelativeContactPosition[b]);

                                // The sign of the change is positive if we're
                                // dealing with the second body in a contact
                                // and negative otherwise (because we're
                                // subtracting the resolution)..
                                c[i].Penetration += DotProduct(deltaPosition, c[i].ContactNormal) * (b ? 1 : -1);
                            }
                        }
                    }
                }
            }
            PositionIterationsUsed++;
        }
    }
}