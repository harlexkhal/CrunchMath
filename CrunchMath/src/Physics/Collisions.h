#pragma once
#include "Contacts.h"

namespace CrunchMath {

    class CollisionDetector;

    class CollisionPrimitive
    {
    public:
        friend class CollisionDetector;

        Body * body;
    };

    class CollisionBox : public CollisionPrimitive
    {
    public:
        Vec3 HalfSize;
    };

    /**
     * A helper structure that contains information for the detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        /**
         * Holds the base of the collision data: the first contact
         * in the array. This is used so that the contact pointer (below)
         * can be incremented each time a contact is detected, while
         * this pointer points to the first contact found.
         */
        Contact *ContactArray;

        /** Holds the contact array to write into. */
        Contact *Contacts;

        /** Holds the maximum number of Contacts the array can take. */
        int ContactsLeft;

        /** Holds the number of Contacts found so far. */
        unsigned ContactCount;

        /** Holds the Friction value to write into any collisions. */
        float Friction;

        /** Holds the Restitution value to write into any collisions. */
        float Restitution;

        /**
         * Holds the collision Tolerance, even uncolliding objects this
         * close should have collisions generated.
         */
        float Tolerance;

        /**
         * Checks if there are more Contacts available in the contact
         * data.
         */
        bool HasMoreContacts()
        {
            return ContactsLeft > 0;
        }

        /**
         * Resets the data so that it has no used Contacts recorded.
         */
        void Reset(unsigned MaxContacts)
        {
            ContactsLeft = MaxContacts;
            ContactCount = 0;
            Contacts = ContactArray;
        }

        /**
         * Notifies the data that the given number of Contacts have
         * been added.
         */
        void AddContacts(unsigned count)
        {
            // Reduce the number of Contacts remaining, add number used
            ContactsLeft -= count;
            ContactCount += count;

            // Move the array forward
            Contacts += count;
        }
    };

    /**
     * A wrapper class that holds the fine grained collision detection
     * routines.
     *
     * Each of the functions has the same format: it takes the details
     * of two objects, and a pointer to a contact array to fill. It
     * returns the number of Contacts it wrote into the array.
     */
    class CollisionDetector
    {
    public:
        static unsigned BoxBox( const CollisionBox &one, const CollisionBox &two, CollisionData *data);
    };
}