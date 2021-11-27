#pragma once
#include "Contacts.h"

namespace CrunchMath {

    class CollisionDetector;

    class cmShape
    {
    public:
        enum Type
        {
            s_Box,
            s_Sphere
        };

        virtual void Set(float x, float y, float z) {};
        virtual void Set(float x) {};
        virtual const void* GetHalfSize() const = 0;
        virtual const Type GetType() const { return m_Shape; };
    protected:
        Type m_Shape;
    };

    class cmBox : public cmShape
    {
    public:
        cmBox()
        {
            m_Shape = s_Box;
        }

        virtual void Set(float x, float y, float z) override 
        {
            HalfSize = Vec3(x, y, z);
        }
        
        virtual const void* GetHalfSize() const override
        {
            return &HalfSize;
        }

    private:
        Vec3 HalfSize;
    };

    class cmSphere : public cmShape
    {
    public:
        cmSphere()
        {
            m_Shape = cmShape::s_Sphere;
            Radius = 0.0f;
        }

        virtual void Set(float r)
        {
            Radius = r;
        }

        virtual const void* GetHalfSize() const override
        {
            return &Radius;
        }

    private:
        float Radius;
    };

    struct CollisionData
    {

        Contact* ptrContactArray;

        Contact* ptrCurrentContact;

        int ContactsSpaceLeft;

        unsigned ContactCount;

        float Friction;

        float Restitution;

        float Tolerance;

        void Reset(unsigned MaxContacts)
        {
            ContactsSpaceLeft = MaxContacts;
            ContactCount = 0;
            ptrCurrentContact = ptrContactArray;
        }

        void AddContacts(unsigned count)
        {
            //Don't add any more contacts if there is no longer space to store new contacts...
            if (ContactsSpaceLeft <= 0)
                return;

            ContactsSpaceLeft -= count;
            ContactCount += count;

            ptrCurrentContact += count;
        }
	};

    class CollisionDetector
    {
    public:
        static unsigned Collision(Body& One, Body& Two, CollisionData* Data);
    };
}