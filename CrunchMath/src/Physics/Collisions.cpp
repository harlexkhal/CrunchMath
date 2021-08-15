#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>
#include "../Math/OBB.h"
#include "Collisions.h"

namespace CrunchMath {

    static inline float TransformToAxis(const CollisionBox& box, const Vec3& axis)
    {
        return
            (
                box.HalfSize.x * fabs(DotProduct(axis, box.body->GetTransform().GetColumnVector(0))) +
                box.HalfSize.y * fabs(DotProduct(axis, box.body->GetTransform().GetColumnVector(1))) +
                box.HalfSize.z * fabs(DotProduct(axis, box.body->GetTransform().GetColumnVector(2)))
            );
    }

    static inline float PenetrationOnAxis(const CollisionBox& One, const CollisionBox& two, const Vec3& axis, const Vec3& toCentre)
    {
        float OneProject = TransformToAxis(One, axis);
        float twoProject = TransformToAxis(two, axis);

        float distance = fabs(DotProduct(toCentre, axis));

        return distance - (OneProject + twoProject);
    }

    static inline bool TryAxis(const CollisionBox& One, const CollisionBox& two, Vec3 axis, const Vec3& toCentre,
        unsigned index, float& SmallestPenetration, unsigned& SmallestCase)
    {
        float Penetration = PenetrationOnAxis(One, two, axis, toCentre);
        if (Penetration > 0)
            return false;

        if (Penetration > SmallestPenetration)
        {
            SmallestPenetration = Penetration;
            SmallestCase = index;
        }

        return true;
    }

    void FillPointFaceBoxBox(const CollisionBox& One, const CollisionBox& two, const Vec3& toCentre, CollisionData* Data, unsigned best, float Pen)
    {
        Contact* contact = Data->Contacts;

        Vec3 normal = One.body->GetTransform().GetColumnVector(best);
        if (DotProduct(One.body->GetTransform().GetColumnVector(best), toCentre) > 0)
        {
            normal = normal * -1.0f;
        }

        Vec3 vertex = two.HalfSize;
        if (DotProduct(two.body->GetTransform().GetColumnVector(0), normal) < 0) vertex.x = -vertex.x;
        if (DotProduct(two.body->GetTransform().GetColumnVector(1), normal) < 0) vertex.y = -vertex.y;
        if (DotProduct(two.body->GetTransform().GetColumnVector(2), normal) < 0) vertex.z = -vertex.z;

        contact->ContactNormal = normal;
        contact->Penetration = Pen;
        contact->ContactPoint = two.body->GetTransform() * vertex;
        contact->setBodyData(One.body, two.body, Data->Friction, Data->Restitution);
    }

    unsigned CollisionDetector::BoxBox(const CollisionBox& One, const CollisionBox& two, CollisionData* Data)
    {
        //I don't think this is necessary ... but i'd just leave it here until i'm ready to optimize the Collision Detection System
        OBB OneOBB(One.body->GetTransform().GetColumnVector(3), One.body->GetTransform(), One.HalfSize);
        OBB TwoOBB(two.body->GetTransform().GetColumnVector(3), two.body->GetTransform(), two.HalfSize);
        if (!OneOBB.BroadPhaseCollisionTest(TwoOBB))
            return 0;

        Vec3 CentreCentreDirection = two.body->GetTransform().GetColumnVector(3) - One.body->GetTransform().GetColumnVector(3);

        float Penetration = -0xfffffffff;
        unsigned BestAxis = 0xffffff;

        //if (!OBBBroadPhase(One, two, Pen, best, toCentre)) return 0;
         
        //Only Supports 2D for now, on 2 Axis
        for (int i = 0; i < 2; i++)
        {
            if (!TryAxis(One, two, One.body->GetTransform().GetColumnVector(i), CentreCentreDirection, (i), Penetration, BestAxis))
                return false;
            if (!TryAxis(One, two, two.body->GetTransform().GetColumnVector(i), CentreCentreDirection, (i + 3), Penetration, BestAxis))
                return false;
        }
        Penetration = fabs(Penetration);

        assert(BestAxis != 0xffffff);
        if (BestAxis < 3)
        {
            FillPointFaceBoxBox(One, two, CentreCentreDirection, Data, BestAxis, Penetration);
            Data->AddContacts(1);
            return 1;
        }

        else if (BestAxis < 6)
        {
            FillPointFaceBoxBox(two, One, CentreCentreDirection * -1.0f, Data, BestAxis - 3, Penetration);
            Data->AddContacts(1);
            return 1;
        }

        return 0;
    }

    /*static inline bool OBBBroadPhase(const CollisionBox& One, const CollisionBox& two, float& Penetration, unsigned& best, Vec3& toCentre)
    {
        bool Collided = false;

        toCentre = two.body->GetTransform().GetColumnVector(3) - One.body->GetTransform().GetColumnVector(3);

        Mat3x3 OneOBBOrientation = One.body->GetTransform();
        Mat3x3 TwoOBBOrientaion = two.body->GetTransform();
        OneOBBOrientation.Transpose();
        TwoOBBOrientaion.Transpose();

        Mat3x3 UnionOBBOrientation = OneOBBOrientation * TwoOBBOrientaion;
        Mat3x3 AbsUnionOBBOrientaion = Abs(UnionOBBOrientation);
        Mat3x3 TransposeAbsUnionOBBOrientation = AbsUnionOBBOrientaion;
        TransposeAbsUnionOBBOrientation.Transpose();

        Vec3 T = OneOBBOrientation * toCentre;
        //T.Normalize();

        Vec3 OneRadius = One.HalfSize;
        Vec3 TwoRadius = TransposeAbsUnionOBBOrientation * two.HalfSize;
        //TwoRadius.Normalize();

        Vec3 FaceA = Abs(T) - (OneRadius + TwoRadius);

        if (FaceA.x < 0.0f && FaceA.y < 0.0f)
        {
            if (FaceA.x > FaceA.y)
            {
                best = 0;
                Penetration = FaceA.x;
                Collided = true;
            }

            else
            {
                best = 1;
                Penetration = FaceA.y;
                Collided = true;
            }
        }

        else
        {
            return false;
        }

        T = TransposeAbsUnionOBBOrientation * toCentre;
        OneRadius = AbsUnionOBBOrientaion * One.HalfSize;
        //OneRadius.Normalize();

        TwoRadius = two.HalfSize;

        Vec3 FaceB = Abs(T) - (OneRadius + TwoRadius);
        if (FaceB.x < 0.0f && FaceB.y < 0.0f)
        {
            if (FaceB.x > FaceB.y)
            {
                if (FaceB.x > (Penetration * .95f))
                {
                    best = 3;
                    Penetration = FaceB.x;
                    Collided = true;
                }
            }

            else
            {
                if (FaceB.y > (Penetration * .95f))
                {
                    best = 4;
                    Penetration = FaceB.y;
                    Collided = true;
                }
            }
        }

        else
        {
            return false;
        }

        return Collided;
    }*/
}