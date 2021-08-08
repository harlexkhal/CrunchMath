#pragma once
#include "Body.h"

namespace CrunchMath {

    /*
     * Forward declaration, see full declaration below for complete
     * documentation.
     */
    class ContactResolver;

    /**
     * A contact represents two bodies in contact. Resolving a
     * contact removes their interPenetration, and applies sufficient
     * impulse to keep them apart. Colliding bodies may also rebound.
     * Contacts can be used to represent Positional joints, by making
     * the contact constraint keep the bodies in their correct
     * Orientation.
     *
     * It can be a good idea to create a contact object even when the
     * contact isn't violated. Because resolving one contact can violate
     * another, Contacts that are close to being violated should be
     * sent to the Resolver; that way if one resolution moves the body,
     * the contact may be violated, and can be resolved. If the contact
     * is not violated, it will not be resolved, so you only loose a
     * small amount of execution time.
     *
     * The contact has no callable functions, it just holds the contact
     * details. To resolve a set of Contacts, use the contact Resolver
     * class.
     */
    class Contact
    {
        // ... Other data as before ...

        /**
         * The contact Resolver object needs access into the Contacts to
         * set and effect the contact.
         */
        friend class ContactResolver;

    public:
        /**
         * Holds the bodies that are involved in the contact. The
         * second of these can be NULL, for Contacts with the scenery.
         */
        Body* body[2];

        /**
         * Holds the lateral Friction coefficient at the contact.
         */
        float Friction;

        /**
         * Holds the normal Restitution coefficient at the contact.
         */
        float Restitution;

        /**
         * Holds the Position of the contact in world coordinates.
         */
        Vec3 ContactPoint;

        /**
         * Holds the direction of the contact in world coordinates.
         */
        Vec3 ContactNormal;

        /**
         * Holds the depth of Penetration at the contact point. If both
         * bodies are specified then the contact point should be midway
         * between the inter-penetrating points.
         */
        float Penetration;

        /**
         * Sets the data that doesn't normally depend on the Position
         * of the contact (i.e. the bodies, and their material properties).
         */
        void setBodyData(Body* one, Body* two, float Friction, float Restitution);

    protected:

        /**
         * A transform matrix that converts co-ordinates in the contact's
         * frame of reference to world co-ordinates. The columns of this
         * matrix form an orthonormal set of vectors.
         */
        Mat3x3 ContactToWorld;

        /**
         * Holds the closing Velocity at the point of contact. This is set
         * when the CalculateInternals function is run.
         */
        Vec3 ContactVelocity;

        /**
         * Holds the required change in Velocity for this contact to be
         * resolved.
         */
        float DesiredDeltaVelocity;

        /**
         * Holds the world space Position of the contact point relative to
         * centre of each body. This is set when the CalculateInternals
         * function is run.
         */
        Vec3 RelativeContactPosition[2];

    protected:
        /**
         * Calculates internal data from state data. This is called before
         * the resolution algorithm tries to do any resolution. It should
         * never need to be called manually.
         */
        void CalculateInternals(float duration);

        /**
         * Reverses the contact. This involves swapping the two rigid bodies
         * and reversing the contact normal. The internal values should then
         * be reCalculated using CalculateInternals (this is not done
         * automatically).
         */
        void SwapBodies();

        /**
         * Updates the awake state of rigid bodies that are taking
         * place in the given contact. A body will be made awake if it
         * is in contact with a body that is awake.
         */
        void MatchAwakeState();

        /**
         * Calculates and sets the internal value for the desired delta
         * Velocity.
         */
        void CalculateDesiredDeltaVelocity(float duration);

        /**
         * Calculates and returns the Velocity of the contact
         * point on the given body.
         */
        Vec3 CalculateLocalVelocity(unsigned bodyIndex, float duration);

        /**
         * Calculates an orthonormal basis for the contact point, based on
         * the primary Friction direction (for anisotropic Friction) or
         * a random Orientation (for isotropic Friction).
         */
        void CalculateContactBasis();

        /**
         * Performs an inertia-weighted impulse based resolution of this
         * contact alone.
         */
        void ApplyVelocityChange(Vec3 VelocityChange[2], Vec3 RotationChange[2]);

        /**
         * Performs an inertia weighted Penetration resolution of this
         * contact alone.
         */
        void ApplyPositionChange(Vec3 linearChange[2], Vec3 angularChange[2], float Penetration);

        /**
         * Calculates the impulse needed to resolve this contact,
         * given that the contact has no Friction. A pair of inertia
         * tensors - one for each contact object - is specified to
         * save calculation time: the calling function has access to
         * these anyway.
         */
        Vec3 CalculateFrictionlessImpulse(Mat3x3 *InverseInertiaTensor);

        /**
         * Calculates the impulse needed to resolve this contact,
         * given that the contact has a non-zero coefficient of
         * Friction. A pair of inertia tensors - one for each contact
         * object - is specified to save calculation time: the calling
         * function has access to these anyway.
         */
        Vec3 CalculateFrictionImpulse(Mat3x3 *InverseInertiaTensor);
    };

    /**
     * The contact resolution routine. One Resolver instance
     * can be shared for the whole simulation, as long as you need
     * roughly the same parameters each time (which is normal).
     *
     * @section algorithm Resolution Algorithm
     *
     * The Resolver uses an iterative satisfaction algorithm; it loops
     * through each contact and tries to resolve it. Each contact is
     * resolved locally, which may in turn put other Contacts in a worse
     * Position. The algorithm then revisits other Contacts and repeats
     * the process up to a specified iteration limit. It can be proved
     * that given enough iterations, the simulation will get to the
     * correct result. As with all approaches, numerical stability can
     * cause problems that make a correct resolution impossible.
     *
     * @subsection strengths Strengths
     *
     * This algorithm is very fast, much faster than other physics
     * approaches. Even using many more iterations than there are
     * Contacts, it will be faster than global approaches.
     *
     * Many global algorithms are unstable under high Friction, this
     * approach is very robust indeed for high Friction and low
     * Restitution values.
     *
     * The algorithm produces visually believable behaviour. Tradeoffs
     * have been made to err on the side of visual floatism rather than
     * computational expense or numerical accuracy.
     *
     * @subsection weaknesses Weaknesses
     *
     * The algorithm does not cope well with situations with many
     * inter-related Contacts: stacked boxes, for example. In this
     * case the simulation may appear to jiggle slightly, which often
     * dislodges a box from the stack, allowing it to collapse.
     *
     * Another issue with the resolution mechanism is that resolving
     * one contact may make another contact move sideways against
     * Friction, because each contact is handled independently, this
     * Friction is not taken into account. If one object is pushing
     * against another, the pushed object may move across its support
     * without Friction, even though Friction is set between those bodies.
     *
     * In general this Resolver is not suitable for stacks of bodies,
     * but is perfect for handling impact, explosive, and flat resting
     * situations.
     */
    class ContactResolver
    {
    protected:
        /**
         * Holds the number of iterations to perform when resolving
         * Velocity.
         */
        unsigned VelocityIterations;

        /**
         * Holds the number of iterations to perform when resolving
         * Position.
         */
        unsigned PositionIterations;

        /**
         * To avoid instability velocities smaller
         * than this value are considered to be zero. Too small and the
         * simulation may be unstable, too large and the bodies may
         * interpenetrate visually. A good starting point is the default
         * of 0.01.
         */
        float VelocityEpsilon;

        /**
         * To avoid instability Penetrations
         * smaller than this value are considered to be not interpenetrating.
         * Too small and the simulation may be unstable, too large and the
         * bodies may interpenetrate visually. A good starting point is
         * the default of0.01.
         */
        float PositionEpsilon;

    public:
        /**
         * Stores the number of Velocity iterations used in the
         * last call to resolve Contacts.
         */
        unsigned VelocityIterationsUsed;

        /**
         * Stores the number of Position iterations used in the
         * last call to resolve Contacts.
         */
        unsigned PositionIterationsUsed;

    public:
        /**
         * Creates a new contact Resolver with the given number of iterations
         * per resolution call, and optional epsilon values.
         */
        ContactResolver(unsigned iterations, float VelocityEpsilon = (float)0.01, float PositionEpsilon = (float)0.01);

        /**
         * Returns true if the Resolver has valid settings and is ready to go.
         */
        bool IsValid()
        {
            return (
                     (VelocityIterations > 0) && (PositionIterations > 0) &&
                      (PositionEpsilon >= 0.0f) && (PositionEpsilon >= 0.0f)
                   );
        }

        /**
         * Sets the number of iterations for each resolution stage.
         */
        void SetIterations(unsigned VelocityIterations, unsigned PositionIterations);

        /**
         * Sets the Tolerance value for both Velocity and Position.
         */
        void SetEpsilon(float VelocityEpsilon, float PositionEpsilon);

        /**
         * Resolves a set of Contacts for both Penetration and Velocity.
         *
         * Contacts that cannot interact with
         * each other should be passed to separate calls to ResolveContacts,
         * as the resolution algorithm takes much longer for lots of
         * Contacts than it does for the same number of Contacts in small
         * sets.
         *
         * @param ContactArray Pointer to an array of contact objects.
         *
         * @param numContacts The number of Contacts in the array to resolve.
         *
         * @param numIterations The number of iterations through the
         * resolution algorithm. This should be at least the number of
         * Contacts (otherwise some constraints will not be resolved -
         * although sometimes this is not noticable). If the iterations are
         * not needed they will not be used, so adding more iterations may
         * not make any difference. In some cases you would need millions
         * of iterations. Think about the number of iterations as a bound:
         * if you specify a large number, sometimes the algorithm WILL use
         * it, and you may drop lots of frames.
         *
         * @param duration The duration of the previous integration step.
         * This is used to compensate for forces applied.
         */
        void ResolveContacts(Contact *ContactArray, unsigned numContacts, float duration);

    protected:
        /**
         * Sets up Contacts ready for processing. This makes sure their
         * internal data is configured correctly and the correct set of bodies
         * is made alive.
         */
        void PrepareContacts(Contact *ContactArray, unsigned numContacts, float duration);

        /**
         * Resolves the Velocity issues with the given array of constraints,
         * using the given number of iterations.
         */
        void AdjustVelocities(Contact *ContactArray, unsigned numContacts, float duration);

        /**
         * Resolves the Positional issues with the given array of constraints,
         * using the given number of iterations.
         */
        void AdjustPositions(Contact *Contacts, unsigned numContacts, float duration);
    };
}