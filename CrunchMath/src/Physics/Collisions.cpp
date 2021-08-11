#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>
#include "Collisions.h"

namespace CrunchMath {

    static inline float TransformToAxis(const CollisionBox& box, const Vec3& axis)
    {
        return
            (
                box.HalfSize.x * fabsf(DotProduct(axis, box.body->GetTransform().GetColumnVector(0))) +
                box.HalfSize.y * fabsf(DotProduct(axis, box.body->GetTransform().GetColumnVector(1))) +
                box.HalfSize.z * fabsf(DotProduct(axis, box.body->GetTransform().GetColumnVector(2)))
            );
    }

    static inline float PenetrationOnAxis(const CollisionBox& one, const CollisionBox& two, const Vec3& axis, const Vec3& toCentre)
    {
        // Project the half-size of one onto axis
        float oneProject = TransformToAxis(one, axis);
        float twoProject = TransformToAxis(two, axis);

        // Project this onto the axis
        float distance = fabsf(DotProduct(toCentre, axis));

        // Return the overlap (i.e. positive indicates
        // overlap, negative indicates separation).
        return oneProject + twoProject - distance;
    }

    static inline bool TryAxis(const CollisionBox& one, const CollisionBox& two, Vec3 axis, const Vec3& toCentre,
        unsigned index, float& smallestPenetration, unsigned& smallestCase)
    {
        // Make sure we have a normalized axis, and don't check almost parallel axes
        float SquareMagnitude = DotProduct(axis, axis);
        if (SquareMagnitude < 0.0001)
            return true;

        axis.Normalize();

        float Penetration = PenetrationOnAxis(one, two, axis, toCentre);
        if (Penetration < 0)
            return false;

        if (Penetration < smallestPenetration)
        {
            smallestPenetration = Penetration;
            smallestCase = index;
        }

        return true;
    }

    void FillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, const Vec3& toCentre, CollisionData* data, unsigned best, float pen)
    {
        // This method is called when we know that a vertex from
        // box two is in contact with box one.

        Contact* contact = data->Contacts;

        // We know which axis the collision is on (i.e. best),
        // but we need to work out which of the two faces on
        // this axis.
        Vec3 normal = one.body->GetTransform().GetColumnVector(best);
        if (DotProduct(one.body->GetTransform().GetColumnVector(best), toCentre) > 0)
        {
            normal = normal * -1.0f;
        }

        // Work out which vertex of box two we're colliding with.
        // Using toCentre doesn't work!
        Vec3 vertex = two.HalfSize;
        if (DotProduct(two.body->GetTransform().GetColumnVector(0), normal) < 0) vertex.x = -vertex.x;
        if (DotProduct(two.body->GetTransform().GetColumnVector(1), normal) < 0) vertex.y = -vertex.y;
        if (DotProduct(two.body->GetTransform().GetColumnVector(2), normal) < 0) vertex.z = -vertex.z;

        // Create the contact data
        contact->ContactNormal = normal;
        contact->Penetration = pen;
        contact->ContactPoint = two.body->GetTransform() * vertex;
        contact->setBodyData(one.body, two.body, data->Friction, data->Restitution);
    }

    unsigned CollisionDetector::BoxBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data)
    {
        //TO Do ----> EarlyOut Intersection Test to be done by math library...
        //if () return 0;

        // Find the vector between the two centres
        Vec3 toCentre = two.body->GetTransform().GetColumnVector(3) - one.body->GetTransform().GetColumnVector(3);

        // We start assuming there is no contact
        float pen = FLT_MAX;
        unsigned best = 0xffffff;

        // Now we check each axes, returning if it gives us
        // a separating axis, and keeping track of the axis with
        // the smallest Penetration otherwise.
        if (!TryAxis(one, two, one.body->GetTransform().GetColumnVector(0), toCentre, (0), pen, best)) return 0;
        if (!TryAxis(one, two, one.body->GetTransform().GetColumnVector(1), toCentre, (1), pen, best)) return 0;
        //if (!TryAxis(one, two, one.getAxis(2), toCentre, (2), pen, best)) return 0;

        if (!TryAxis(one, two, one.body->GetTransform().GetColumnVector(0), toCentre, (3), pen, best)) return 0;
        if (!TryAxis(one, two, one.body->GetTransform().GetColumnVector(1), toCentre, (4), pen, best)) return 0;
        //if (!TryAxis(one, two, one.getAxis(2), toCentre, (5), pen, best)) return 0;

        // Store the best axis-major, in case we run into almost
        // parallel edge collisions later
        unsigned bestSingleAxis = best;

        // Make sure we've got a result.
        assert(best != 0xffffff);

        if (best < 3)
        {
            FillPointFaceBoxBox(one, two, toCentre, data, best, pen);
            data->AddContacts(1);
            return 1;
        }

        else if (best < 6)
        {
            FillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
            data->AddContacts(1);
            return 1;
        }

        return 0;
    }
}