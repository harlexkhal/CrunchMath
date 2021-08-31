#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>
#include "../Math/OBB.h"
#include "Collisions.h"

namespace CrunchMath {

    static inline float TransformToAxis(const Shape& box, const Vec3& axis)
    {
        Vec3 HalfSize = box.GetHalfSize();
        return
            (
                HalfSize.x * fabs(DotProduct(axis, box.GetBody()->GetTransform().GetColumnVector(0))) +
                HalfSize.y * fabs(DotProduct(axis, box.GetBody()->GetTransform().GetColumnVector(1))) +
                HalfSize.z * fabs(DotProduct(axis, box.GetBody()->GetTransform().GetColumnVector(2)))
            );
    }

    static inline float PenetrationOnAxis(const Shape& One, const Shape& Two, const Vec3& axis, const Vec3& toCentre)
    {
        float OneProject = TransformToAxis(One, axis);
        float TwoProject = TransformToAxis(Two, axis);

        float distance = fabs(DotProduct(toCentre, axis));

        return distance - (OneProject + TwoProject);
    }

    static inline bool TryAxis(const Shape& One, const Shape& Two, Vec3 axis, const Vec3& toCentre,
        unsigned index, float& SmallestPenetration, unsigned& SmallestCase)
    {
        float Penetration = PenetrationOnAxis(One, Two, axis, toCentre);
        if (Penetration > 0)
            return false;

        if (Penetration > SmallestPenetration)
        {
            SmallestPenetration = Penetration;
            SmallestCase = index;
        }

        return true;
    }

    void FillPointFaceBoxBox(const Shape* One, const Shape* Two, const Vec3& toCentre, CollisionData* Data, unsigned best, float Pen)
    {
        Contact* contact = Data->Contacts;

        Vec3 normal = One->GetBody()->GetTransform().GetColumnVector(best);
        if (DotProduct(One->GetBody()->GetTransform().GetColumnVector(best), toCentre) > 0)
        {
            normal = normal * -1.0f;
        }

        Vec3 vertex = Two->GetHalfSize();
        if (DotProduct(Two->GetBody()->GetTransform().GetColumnVector(0), normal) < 0) vertex.x = -vertex.x;
        if (DotProduct(Two->GetBody()->GetTransform().GetColumnVector(1), normal) < 0) vertex.y = -vertex.y;
        if (DotProduct(Two->GetBody()->GetTransform().GetColumnVector(2), normal) < 0) vertex.z = -vertex.z;

        contact->ContactNormal = normal;
        contact->Penetration = Pen;
        contact->ContactPoint = Two->GetBody()->GetTransform() * vertex;
        contact->setBodyData(One->GetBody(), Two->GetBody(), Data->Friction, Data->Restitution);
    }

    unsigned CollisionDetector::Collision(const Shape* One, const Shape* Two, CollisionData* Data)
    {
        //I don't think this is necessary ... but i'd just leave it here until i'm ready to optimize the Collision Detection System
       // OBB OneOBB(One->body->GetTransform().GetColumnVector(3), One->body->GetTransform(), One->HalfSize);
       // OBB TwoOBB(Two->body->GetTransform().GetColumnVector(3), Two->body->GetTransform(), Two->HalfSize);
       // if (!OneOBB.BroadPhaseCollisionTest(TwoOBB))
        //    return 0;

        Vec3 CentreCentreDirection = Two->GetBody()->GetTransform().GetColumnVector(3) - One->GetBody()->GetTransform().GetColumnVector(3);

        float Penetration = -0xfffffffff;
        unsigned BestAxis = 0xffffff;
         
        //Only Supports 2D for now, on 2 Axis
        for (int i = 0; i < 2; i++)
        {
            if (!TryAxis(*One, *Two, One->GetBody()->GetTransform().GetColumnVector(i), CentreCentreDirection, (i), Penetration, BestAxis))
                return false;
            if (!TryAxis(*One, *Two, Two->GetBody()->GetTransform().GetColumnVector(i), CentreCentreDirection, (i + 3), Penetration, BestAxis))
                return false;
        }
        Penetration = fabs(Penetration);

        assert(BestAxis != 0xffffff);
        if (BestAxis < 3)
        {
            FillPointFaceBoxBox(One, Two, CentreCentreDirection, Data, BestAxis, Penetration);
            Data->AddContacts(1);
            return 1;
        }

        else if (BestAxis < 6)
        {
            FillPointFaceBoxBox(Two, One, CentreCentreDirection * -1.0f, Data, BestAxis - 3, Penetration);
            Data->AddContacts(1);
            return 1;
        }

        return 0;
    }
}