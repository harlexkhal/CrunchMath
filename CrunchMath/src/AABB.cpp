#include "AABB.h"
#include "Mat4x4.h"

namespace CrunchMath {

    AABB::AABB()
    {
        for (int i = 0; i < 3; i++)
        {
            this->Min[i] = 0.0f;
            this->Max[i] = 0.0f;
        }
    }

    AABB::AABB(const Vec3& Min, const Vec3& Max)
    {
        this->Set(Min, Max);
    }

    AABB::AABB(const AABB& Box)
    {
        for (int i = 0; i < 3; i++)
        {
            this->Min[i] = Box.Min[i];
            this->Max[i] = Box.Max[i];
        }  
    }

    AABB& AABB::operator=(const AABB& Box)
    {
        for (int i = 0; i < 3; i++)
        {
            this->Min[i] = Box.Min[i];
            this->Max[i] = Box.Max[i];
        }
        return *this;
    }

    Vec3 AABB::ClosestPointAABBPt(const Vec3& Point) const
    {
        Vec3 ClosestPointOnAABBToPoint = Point;

            if (Point.x < Min[0])
            {
                ClosestPointOnAABBToPoint.x = Min[0];
            }

            else if (Point.x > Max[0])
            {
                ClosestPointOnAABBToPoint.x = Max[0];
            }

            if (Point.y < Min[1])
            {
                ClosestPointOnAABBToPoint.y = Min[1];
            }

            else if (Point.y > Max[1])
            {
                ClosestPointOnAABBToPoint.y = Max[1];
            }

            if (Point.z < Min[2])
            {
                ClosestPointOnAABBToPoint.z = Min[2];
            }

            else if (Point.z > Max[2])
            {
                ClosestPointOnAABBToPoint.z = Max[2];
            }

            return ClosestPointOnAABBToPoint;
    }

    bool AABB::ALTIntersectTest(const AABB& Object2)
    {
        Vec3 _thisCenterPoint;
        Vec3 Object2CenterPoint_;
       
        _thisCenterPoint.x = (Min[0] + Max[0]) / 2.0f;
        _thisCenterPoint.y = (Min[1] + Max[1]) / 2.0f;
        _thisCenterPoint.z = (Min[2] + Max[2]) / 2.0f;

        Object2CenterPoint_.x = (Object2.Min[0] + Object2.Max[0]) / 2.0f;
        Object2CenterPoint_.y = (Object2.Min[1] + Object2.Max[1]) / 2.0f;
        Object2CenterPoint_.z = (Object2.Min[2] + Object2.Max[2]) / 2.0f;

        //To get the radius of Object2->AABB with respect to the closest point on Object2->AABB to this->Object center
        Vec3 ClosestPtOnObject2 = Object2.ClosestPointAABBPt(_thisCenterPoint);
        /*using Distance() func not recommended because of Sqrt operation >>> slows down program.
        never use a sqrt() if you won't need it. since all we want to do is compare Distance's*/
        //float RadiusOfObject2 = Distance(Object2CenterPoint_, ClosestPtOnObject2);  //for better performance don't use Distance(Object2CenterPoint_, ClosestPtOnObject2) to compute RadiusOfObject2
        float RadiusOfObject2 = DotProduct(Object2CenterPoint_ - ClosestPtOnObject2, Object2CenterPoint_ - ClosestPtOnObject2); //Gives the Squared Distance Object2CenterPoint_ [to] ClosestPtOnObject2

        //Getting closest  point on this->AABB to closest radius extent point on Object2->AABB
        Vec3 ClosestPointOn_this_AABB = ClosestPointAABBPt(ClosestPtOnObject2);

        //Distance between closest point ClosestPointOn_this_AABB [to]  Object2CenterPoint_(Object2->AABB center point)
        /*using Distance() func not recommended because of Sqrt operation >>> slows down program.
        never use a sqrt() if you won't need it. since all we want to do is compare Distance's*/
        //float D = Distance(ClosestPointOn_this_AABB, Object2CenterPoint_); //for better performance don't use Distance(ClosestPointOn_this_AABB, Object2CenterPoint_) to compute D
        float D = DotProduct(ClosestPointOn_this_AABB - Object2CenterPoint_, ClosestPointOn_this_AABB - Object2CenterPoint_);

        /*Note : if using Squared Distance(DotProduct func) both RadiusOfObject2 and D should be in Squared Distance,
        if using actual Distance(Distance func) both RadiusOfObject2 and D should be in Actual Distance. any other way
        makes the whole query read wrongly*/

        //Reports as intersecting if Radius(Object2CenterPoint_- ClosestPtOnObject2) becomes > or = the Distance (ClosestPointOn_this_AABB-Object2CenterPoint_)
        return (RadiusOfObject2 >= D);
    }

    bool AABB::IntersectTest(const AABB& Box)
    {
        bool Collision[3];
        for (int i = 0; i < 3; i++)
        {
            Collision[i] = this->Min[i] <= Box.Max[i] && this->Max[i] >= Box.Min[i];
        }
       
        return (Collision[0] && Collision[1] && Collision[2]);
    }

    void AABB::Set(const Vec3& Min, const Vec3& Max)
    {
        this->Min[0] = Min.x;
        this->Min[1] = Min.y;
        this->Min[2] = Min.z;

        this->Max[0] = Max.x;
        this->Max[1] = Max.y;
        this->Max[2] = Max.z;
    }

   /* void AABB::TransformBox(const Quaternion& Rotation, const Vec3& Position, Vec3 Size)
    {
        Mat4x4 newBoxModel(1.0f);
        newBoxModel.Rotate(Vec3(Rotation.x, Rotation.y, Rotation.z), Rotation.w);
        //---NewBoxModel Scaled in a way to still Align box with ObjectModel---
        newBoxModel.Scale((Size * 0.5f * 0.5f));

        AABB UpdatedBox;
        UpdatedBox.Min[0] = Min[0]; UpdatedBox.Max[0] = Max[0];
        UpdatedBox.Min[1] = Min[1]; UpdatedBox.Max[1] = Max[1];
        UpdatedBox.Min[2] = Min[2]; UpdatedBox.Max[2] = Max[2];

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                float e = newBoxModel.Matrix[j][i] * this->Min[j];
                float f = newBoxModel.Matrix[j][i] * this->Max[j];

                if (e > f)
                {
                    UpdatedBox.Min[j] += f;
                    UpdatedBox.Max[j] += e;
                }

                else
                {
                    UpdatedBox.Min[j] += e;
                    UpdatedBox.Max[j] += f;
                }    
            }
        }
        *this = UpdatedBox;
    }*/
}