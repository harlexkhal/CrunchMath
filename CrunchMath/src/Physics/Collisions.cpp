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

    /*
     * This function checks if the two boxes overlap
     * along the given axis, returning the ammount of overlap.
     * The final parameter toCentre
     * is used to pass in the vector between the boxes centre
     * points, to avoid having to reCalculate it each time.
     */
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

    static inline Vec3 ContactPoint(const Vec3& pOne, const Vec3& dOne, float oneSize, const Vec3& pTwo,
                                                            const Vec3& dTwo, float twoSize, bool useOne)
    {
        Vec3 toSt, cOne, cTwo;
        float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
        float denom, mua, mub;

        float dOne_SquareMagnitude = DotProduct(dOne, dOne);
        float dTwo_SquareMagnitude = DotProduct(dTwo, dTwo);

        smOne = dOne_SquareMagnitude;
        smTwo = dTwo_SquareMagnitude;

        dpOneTwo = DotProduct(dTwo, dOne);

        toSt = pOne - pTwo;
        dpStaOne = DotProduct(dOne, toSt);
        dpStaTwo = DotProduct(dTwo, toSt);

        denom = smOne * smTwo - dpOneTwo * dpOneTwo;

        // Zero denominator indicates parrallel lines
        if (fabsf(denom) < 0.0001f)
        {
            return useOne ? pOne : pTwo;
        }

        mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
        mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

        // If either of the edges has the nearest point out
        // of bounds, then the edges aren't crossed, we have
        // an edge-face contact. Our point is on the edge, which
        // we know from the useOne parameter.
        if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
        {
            return useOne ? pOne : pTwo;
        }

        else
        {
            cOne = pOne + dOne * mua;
            cTwo = pTwo + dTwo * mub;

            return cOne * 0.5 + cTwo * 0.5;
        }
    }

    unsigned CollisionDetector::BoxBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data)
    {
        //TO Do ----> EarlyOut Intersection Test to be done by math library...
        //if (!IntersectionTests::boxAndBox(one, two)) return 0;

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

        /*CHECK_OVERLAP(one.getAxis(0) % two.getAxis(0), 6);
        CHECK_OVERLAP(one.getAxis(0) % two.getAxis(1), 7);
        CHECK_OVERLAP(one.getAxis(0) % two.getAxis(2), 8);
        CHECK_OVERLAP(one.getAxis(1) % two.getAxis(0), 9);
        CHECK_OVERLAP(one.getAxis(1) % two.getAxis(1), 10);
        CHECK_OVERLAP(one.getAxis(1) % two.getAxis(2), 11);
        CHECK_OVERLAP(one.getAxis(2) % two.getAxis(0), 12);
        CHECK_OVERLAP(one.getAxis(2) % two.getAxis(1), 13);
        CHECK_OVERLAP(one.getAxis(2) % two.getAxis(2), 14);*/

        // Make sure we've got a result.
        assert(best != 0xffffff);

        // We now know there's a collision, and we know which
        // of the axes gave the smallest Penetration. We now
        // can deal with it in different ways depending on
        // the case.
        if (best < 3)
        {
            // We've got a vertex of box two on a face of box one.
            FillPointFaceBoxBox(one, two, toCentre, data, best, pen);
            data->AddContacts(1);
            return 1;
        }

        else if (best < 6)
        {
            // We've got a vertex of box one on a face of box two.
            // We use the same algorithm as above, but swap around
            // one and two (and therefore also the vector between their
            // centres).
            FillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
            data->AddContacts(1);
            return 1;
        }

        else
        {
            // We've got an edge-edge contact. Find out which axes
            best -= 6;
            unsigned oneAxisIndex = best / 3;
            unsigned twoAxisIndex = best % 3;
            Vec3 oneAxis = one.body->GetTransform().GetColumnVector(oneAxisIndex);
            Vec3 twoAxis = two.body->GetTransform().GetColumnVector(twoAxisIndex);
            Vec3 axis = CrossProduct(oneAxis, twoAxis);
            axis.Normalize();

            // The axis should point from box one to box two.
            if (DotProduct(axis, toCentre) > 0) axis = axis * -1.0f;

            // We have the axes, but not the edges: each axis has 4 edges parallel
            // to it, we need to find which of the 4 for each object. We do
            // that by finding the point in the centre of the edge. We know
            // its component in the direction of the box's collision axis is zero
            // (its a mid-point) and we determine which of the extremes in each
            // of the other axes is closest.
            Vec3 ptOnOneEdge = one.HalfSize;
            Vec3 ptOnTwoEdge = two.HalfSize;
            for (unsigned i = 0; i < 3; i++)
            {
                if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
                else if (DotProduct(one.body->GetTransform().GetColumnVector(i), axis) > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

                if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
                else if (DotProduct(two.body->GetTransform().GetColumnVector(i), axis) < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
            }

            // Move them into world coordinates (they are already oriented
            // correctly, since they have been derived from the axes).
            ptOnOneEdge = one.body->GetTransform() * ptOnOneEdge;
            ptOnTwoEdge = two.body->GetTransform() * ptOnTwoEdge;

            // So we have a point and a direction for the colliding edges.
            // We need to find out point of closest approach of the two
            // line-segments.
            Vec3 vertex = ContactPoint(
                                         ptOnOneEdge, oneAxis, one.HalfSize[oneAxisIndex],
                                         ptOnTwoEdge, twoAxis, two.HalfSize[twoAxisIndex],
                                         bestSingleAxis > 2
                                      );

            // We can fill the contact.
            Contact* contact = data->Contacts;

            contact->Penetration = pen;
            contact->ContactNormal = axis;
            contact->ContactPoint = vertex;
            contact->setBodyData(one.body, two.body, data->Friction, data->Restitution);
            data->AddContacts(1);
            return 1;
        }

        return 0;
    }
}