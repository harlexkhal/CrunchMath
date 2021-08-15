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

    struct CollisionData
    {
        Contact *ContactArray;

        Contact *Contacts;

        int ContactsLeft;

        unsigned ContactCount;

        float Friction;

        float Restitution;

        float Tolerance;

        bool HasMoreContacts()
        {
            return ContactsLeft > 0;
        }

        void Reset(unsigned MaxContacts)
        {
            ContactsLeft = MaxContacts;
            ContactCount = 0;
            Contacts = ContactArray;
        }

        void AddContacts(unsigned count)
        {
            ContactsLeft -= count;
            ContactCount += count;

            Contacts += count;
        }
    };

    class CollisionDetector
    {
    public:
        static unsigned BoxBox( const CollisionBox &one, const CollisionBox &two, CollisionData *data);
    };
}