#pragma once
#include "Contacts.h"

namespace CrunchMath {

    class CollisionDetector;

    class Shape
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

    class Box : public Shape
    {
    public:
        Box()
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

    class cmSphere : public Shape
    {
    public:
        cmSphere()
        {
            m_Shape = Shape::s_Sphere;
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
        static unsigned Collision(Body* one, Body* two, CollisionData *data);
    };
}