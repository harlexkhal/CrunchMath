#pragma once
#include "body.h"

namespace CrunchPhysx {

    /*
     * Forward declaration, see full declaration below for complete
     * documentation.
     */
    class ContactResolver;

    /**
     * A contact represents two bodies in contact. Resolving a
     * contact removes their interpenetration, and applies sufficient
     * impulse to keep them apart. Colliding bodies may also rebound.
     * Contacts can be used to represent positional joints, by making
     * the contact constraint keep the bodies in their correct
     * orientation.
     *
     * It can be a good idea to create a contact object even when the
     * contact isn't violated. Because resolving one contact can violate
     * another, Contacts that are close to being violated should be
     * sent to the resolver; that way if one resolution moves the body,
     * the contact may be violated, and can be resolved. If the contact
     * is not violated, it will not be resolved, so you only loose a
     * small amount of execution time.
     *
     * The contact has no callable functions, it just holds the contact
     * details. To resolve a set of Contacts, use the contact resolver
     * class.
     */
    class Contact
    {
        // ... Other data as before ...

        /**
         * The contact resolver object needs access into the Contacts to
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
         * Holds the lateral friction coefficient at the contact.
         */
        cpfloat friction;

        /**
         * Holds the normal restitution coefficient at the contact.
         */
        cpfloat restitution;

        /**
         * Holds the position of the contact in world coordinates.
         */
        Vector3 contactPoint;

        /**
         * Holds the direction of the contact in world coordinates.
         */
        Vector3 contactNormal;

        /**
         * Holds the depth of penetration at the contact point. If both
         * bodies are specified then the contact point should be midway
         * between the inter-penetrating points.
         */
        cpfloat penetration;

        /**
         * Sets the data that doesn't normally depend on the position
         * of the contact (i.e. the bodies, and their material properties).
         */
        void setBodyData(Body* one, Body* two, cpfloat friction, cpfloat restitution);

    protected:

        /**
         * A transform matrix that converts co-ordinates in the contact's
         * frame of reference to world co-ordinates. The columns of this
         * matrix form an orthonormal set of vectors.
         */
        Matrix3 contactToWorld;

        /**
         * Holds the closing velocity at the point of contact. This is set
         * when the calculateInternals function is run.
         */
        Vector3 contactVelocity;

        /**
         * Holds the required change in velocity for this contact to be
         * resolved.
         */
        cpfloat desiredDeltaVelocity;

        /**
         * Holds the world space position of the contact point relative to
         * centre of each body. This is set when the calculateInternals
         * function is run.
         */
        Vector3 relativeContactPosition[2];

    protected:
        /**
         * Calculates internal data from state data. This is called before
         * the resolution algorithm tries to do any resolution. It should
         * never need to be called manually.
         */
        void calculateInternals(cpfloat duration);

        /**
         * Reverses the contact. This involves swapping the two rigid bodies
         * and reversing the contact normal. The internal values should then
         * be recalculated using calculateInternals (this is not done
         * automatically).
         */
        void swapBodies();

        /**
         * Updates the awake state of rigid bodies that are taking
         * place in the given contact. A body will be made awake if it
         * is in contact with a body that is awake.
         */
        void matchAwakeState();

        /**
         * Calculates and sets the internal value for the desired delta
         * velocity.
         */
        void calculateDesiredDeltaVelocity(cpfloat duration);

        /**
         * Calculates and returns the velocity of the contact
         * point on the given body.
         */
        Vector3 calculateLocalVelocity(unsigned bodyIndex, cpfloat duration);

        /**
         * Calculates an orthonormal basis for the contact point, based on
         * the primary friction direction (for anisotropic friction) or
         * a random orientation (for isotropic friction).
         */
        void calculateContactBasis();

        /**
         * Performs an inertia-weighted impulse based resolution of this
         * contact alone.
         */
        void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);

        /**
         * Performs an inertia weighted penetration resolution of this
         * contact alone.
         */
        void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], cpfloat penetration);

        /**
         * Calculates the impulse needed to resolve this contact,
         * given that the contact has no friction. A pair of inertia
         * tensors - one for each contact object - is specified to
         * save calculation time: the calling function has access to
         * these anyway.
         */
        Vector3 calculateFrictionlessImpulse(Matrix3 *inverseInertiaTensor);

        /**
         * Calculates the impulse needed to resolve this contact,
         * given that the contact has a non-zero coefficient of
         * friction. A pair of inertia tensors - one for each contact
         * object - is specified to save calculation time: the calling
         * function has access to these anyway.
         */
        Vector3 calculateFrictionImpulse(Matrix3 *inverseInertiaTensor);
    };

    /**
     * The contact resolution routine. One resolver instance
     * can be shared for the whole simulation, as long as you need
     * roughly the same parameters each time (which is normal).
     *
     * @section algorithm Resolution Algorithm
     *
     * The resolver uses an iterative satisfaction algorithm; it loops
     * through each contact and tries to resolve it. Each contact is
     * resolved locally, which may in turn put other Contacts in a worse
     * position. The algorithm then revisits other Contacts and repeats
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
     * Many global algorithms are unstable under high friction, this
     * approach is very robust indeed for high friction and low
     * restitution values.
     *
     * The algorithm produces visually believable behaviour. Tradeoffs
     * have been made to err on the side of visual cpfloatism rather than
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
     * friction, because each contact is handled independently, this
     * friction is not taken into account. If one object is pushing
     * against another, the pushed object may move across its support
     * without friction, even though friction is set between those bodies.
     *
     * In general this resolver is not suitable for stacks of bodies,
     * but is perfect for handling impact, explosive, and flat resting
     * situations.
     */
    class ContactResolver
    {
    protected:
        /**
         * Holds the number of iterations to perform when resolving
         * velocity.
         */
        unsigned velocityIterations;

        /**
         * Holds the number of iterations to perform when resolving
         * position.
         */
        unsigned positionIterations;

        /**
         * To avoid instability velocities smaller
         * than this value are considered to be zero. Too small and the
         * simulation may be unstable, too large and the bodies may
         * interpenetrate visually. A good starting point is the default
         * of 0.01.
         */
        cpfloat velocityEpsilon;

        /**
         * To avoid instability penetrations
         * smaller than this value are considered to be not interpenetrating.
         * Too small and the simulation may be unstable, too large and the
         * bodies may interpenetrate visually. A good starting point is
         * the default of0.01.
         */
        cpfloat positionEpsilon;

    public:
        /**
         * Stores the number of velocity iterations used in the
         * last call to resolve Contacts.
         */
        unsigned velocityIterationsUsed;

        /**
         * Stores the number of position iterations used in the
         * last call to resolve Contacts.
         */
        unsigned positionIterationsUsed;

    private:
        /**
         * Keeps track of whether the internal settings are valid.
         */
        bool validSettings;

    public:
        /**
         * Creates a new contact resolver with the given number of iterations
         * per resolution call, and optional epsilon values.
         */
        ContactResolver(unsigned iterations, cpfloat velocityEpsilon = (cpfloat)0.01, cpfloat positionEpsilon = (cpfloat)0.01);

        /**
         * Returns true if the resolver has valid settings and is ready to go.
         */
        bool isValid()
        {
            return (
                     (velocityIterations > 0) && (positionIterations > 0) &&
                      (positionEpsilon >= 0.0f) && (positionEpsilon >= 0.0f)
                   );
        }

        /**
         * Sets the number of iterations for each resolution stage.
         */
        void setIterations(unsigned velocityIterations, unsigned positionIterations);

        /**
         * Sets the tolerance value for both velocity and position.
         */
        void setEpsilon(cpfloat velocityEpsilon, cpfloat positionEpsilon);

        /**
         * Resolves a set of Contacts for both penetration and velocity.
         *
         * Contacts that cannot interact with
         * each other should be passed to separate calls to resolveContacts,
         * as the resolution algorithm takes much longer for lots of
         * Contacts than it does for the same number of Contacts in small
         * sets.
         *
         * @param contactArray Pointer to an array of contact objects.
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
        void resolveContacts(Contact *contactArray, unsigned numContacts, cpfloat duration);

    protected:
        /**
         * Sets up Contacts ready for processing. This makes sure their
         * internal data is configured correctly and the correct set of bodies
         * is made alive.
         */
        void prepareContacts(Contact *contactArray, unsigned numContacts, cpfloat duration);

        /**
         * Resolves the velocity issues with the given array of constraints,
         * using the given number of iterations.
         */
        void adjustVelocities(Contact *contactArray, unsigned numContacts, cpfloat duration);

        /**
         * Resolves the positional issues with the given array of constraints,
         * using the given number of iterations.
         */
        void adjustPositions(Contact *Contacts, unsigned numContacts, cpfloat duration);
    };
}