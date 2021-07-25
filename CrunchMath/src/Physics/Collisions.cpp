#include "Collisions.h"
#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>

using namespace CrunchPhysx;

static inline cpfloat transformToAxis(const CollisionBox &box, const Vector3 &axis)
{
    return
        (
            box.halfSize.x * cp_abs(axis * box.body->getTransform().getAxisVector(0)) +
            box.halfSize.y * cp_abs(axis * box.body->getTransform().getAxisVector(1)) +
            box.halfSize.z * cp_abs(axis * box.body->getTransform().getAxisVector(2))
        );
}

/*
 * This function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline cpfloat penetrationOnAxis(const CollisionBox &one, const CollisionBox &two, const Vector3 &axis, const Vector3 &toCentre)
{
    // Project the half-size of one onto axis
    cpfloat oneProject = transformToAxis(one, axis);
    cpfloat twoProject = transformToAxis(two, axis);

    // Project this onto the axis
    cpfloat distance = cp_abs(toCentre * axis);

    // Return the overlap (i.e. positive indicates
    // overlap, negative indicates separation).
    return oneProject + twoProject - distance;
}

static inline bool tryAxis(const CollisionBox &one, const CollisionBox &two, Vector3 axis, const Vector3& toCentre,
                            unsigned index, cpfloat& smallestPenetration, unsigned &smallestCase)
{
    // Make sure we have a normalized axis, and don't check almost parallel axes
    if (axis.squareMagnitude() < 0.0001) 
        return true;

    axis.normalise();

    cpfloat penetration = penetrationOnAxis(one, two, axis, toCentre);
    if (penetration < 0)
        return false;

    if (penetration < smallestPenetration)
    {
        smallestPenetration = penetration;
        smallestCase = index;
    }

    return true;
}

void fillPointFaceBoxBox(const CollisionBox &one, const CollisionBox &two, const Vector3 &toCentre, 
                                                      CollisionData *data, unsigned best, cpfloat pen)
{
    // This method is called when we know that a vertex from
    // box two is in contact with box one.

    Contact* contact = data->Contacts;

    // We know which axis the collision is on (i.e. best),
    // but we need to work out which of the two faces on
    // this axis.
    Vector3 normal = one.body->getTransform().getAxisVector(best);
    if (one.body->getTransform().getAxisVector(best) * toCentre > 0)
    {
        normal = normal * -1.0f;
    }

    // Work out which vertex of box two we're colliding with.
    // Using toCentre doesn't work!
    Vector3 vertex = two.halfSize;
    if (two.body->getTransform().getAxisVector(0) * normal < 0) vertex.x = -vertex.x;
    if (two.body->getTransform().getAxisVector(1) * normal < 0) vertex.y = -vertex.y;
    if (two.body->getTransform().getAxisVector(2) * normal < 0) vertex.z = -vertex.z;

    // Create the contact data
    contact->contactNormal = normal;
    contact->penetration = pen;
    contact->contactPoint = two.body->getTransform() * vertex;
    contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

static inline Vector3 contactPoint(const Vector3 &pOne, const Vector3 &dOne, cpfloat oneSize, const Vector3 &pTwo, 
                                                                const Vector3 &dTwo, cpfloat twoSize, bool useOne)
{
    Vector3 toSt, cOne, cTwo;
    cpfloat dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
    cpfloat denom, mua, mub;

    smOne = dOne.squareMagnitude();
    smTwo = dTwo.squareMagnitude();
    dpOneTwo = dTwo * dOne;

    toSt = pOne - pTwo;
    dpStaOne = dOne * toSt;
    dpStaTwo = dTwo * toSt;

    denom = smOne * smTwo - dpOneTwo * dpOneTwo;

    // Zero denominator indicates parrallel lines
    if (cp_abs(denom) < 0.0001f) 
    {
        return useOne?pOne:pTwo;
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

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::BoxBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data)
{
    //TO Do ----> EarlyOut Intersection Test to be done by math library...
    //if (!IntersectionTests::boxAndBox(one, two)) return 0;

    // Find the vector between the two centres
    Vector3 toCentre = two.body->getTransform().getAxisVector(3) - one.body->getTransform().getAxisVector(3);

    // We start assuming there is no contact
    cpfloat pen = cp_MAX;
    unsigned best = 0xffffff;

    // Now we check each axes, returning if it gives us
    // a separating axis, and keeping track of the axis with
    // the smallest penetration otherwise.
    if (!tryAxis(one, two, one.body->getTransform().getAxisVector(0), toCentre, (0), pen, best)) return 0;
    if (!tryAxis(one, two, one.body->getTransform().getAxisVector(1), toCentre, (1), pen, best)) return 0;
    //if (!tryAxis(one, two, one.getAxis(2), toCentre, (2), pen, best)) return 0;

    if (!tryAxis(one, two, one.body->getTransform().getAxisVector(0), toCentre, (3), pen, best)) return 0;
    if (!tryAxis(one, two, one.body->getTransform().getAxisVector(1), toCentre, (4), pen, best)) return 0;
    //if (!tryAxis(one, two, one.getAxis(2), toCentre, (5), pen, best)) return 0;

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
    // of the axes gave the smallest penetration. We now
    // can deal with it in different ways depending on
    // the case.
    if (best < 3)
    {
        // We've got a vertex of box two on a face of box one.
        fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
        data->addContacts(1);
        return 1;
    }

    else if (best < 6)
    {
        // We've got a vertex of box one on a face of box two.
        // We use the same algorithm as above, but swap around
        // one and two (and therefore also the vector between their
        // centres).
        fillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best-3, pen);
        data->addContacts(1);
        return 1;
    }

    else
    {
        // We've got an edge-edge contact. Find out which axes
        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = one.body->getTransform().getAxisVector(oneAxisIndex);
        Vector3 twoAxis = two.body->getTransform().getAxisVector(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.normalise();

        // The axis should point from box one to box two.
        if (axis * toCentre > 0) axis = axis * -1.0f;

        // We have the axes, but not the edges: each axis has 4 edges parallel
        // to it, we need to find which of the 4 for each object. We do
        // that by finding the point in the centre of the edge. We know
        // its component in the direction of the box's collision axis is zero
        // (its a mid-point) and we determine which of the extremes in each
        // of the other axes is closest.
        Vector3 ptOnOneEdge = one.halfSize;
        Vector3 ptOnTwoEdge = two.halfSize;
        for (unsigned i = 0; i < 3; i++)
        {
            if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
            else if (one.body->getTransform().getAxisVector(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (two.body->getTransform().getAxisVector(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
        }

        // Move them into world coordinates (they are already oriented
        // correctly, since they have been derived from the axes).
        ptOnOneEdge = one.body->getTransform() * ptOnOneEdge;
        ptOnTwoEdge = two.body->getTransform() * ptOnTwoEdge;

        // So we have a point and a direction for the colliding edges.
        // We need to find out point of closest approach of the two
        // line-segments.
        Vector3 vertex = contactPoint(
            ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
            ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
            bestSingleAxis > 2
            );

        // We can fill the contact.
        Contact* contact = data->Contacts;

        contact->penetration = pen;
        contact->contactNormal = axis;
        contact->contactPoint = vertex;
        contact->setBodyData(one.body, two.body,
            data->friction, data->restitution);
        data->addContacts(1);
        return 1;
    }
    return 0;
}