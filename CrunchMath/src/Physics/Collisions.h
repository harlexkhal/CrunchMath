#pragma once
#include "Contacts.h"

namespace CrunchMath {

    class CollisionDetector;

    class Shape
    {
    public:
        virtual void SetHalfExtent(float x, float y, float z) = 0;
        virtual void SetBody(Body* body) = 0;
        virtual Body* GetBody() const = 0;
        virtual const Vec3& GetHalfSize() const = 0;
    protected:
        Vec3 HalfSize;
        Body* body;
    };

    class Box : public Shape
    {
    public:
        virtual void SetHalfExtent(float x, float y, float z) override 
        {
            HalfSize = Vec3(x, y, z);
        }

        virtual void SetBody(Body* body) override
        {
            this->body = body;
        }

        virtual Body* GetBody() const override
        {
            return body;
        }
        
        virtual const Vec3& GetHalfSize() const override
        {
            return HalfSize;
        }
    };

    class cmSphere : public Shape
    {
    public:
        void SetRadius3D(float r)
        {
            HalfSize = Vec3(r, r, r);
        }

        void SetRadius2D(float r)
        {
            HalfSize = Vec3(r, r, 0.0f);
        }
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
        static unsigned Collision(const Shape* one, const Shape* two, CollisionData *data);
    };
}