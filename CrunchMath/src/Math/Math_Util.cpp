#include <cmath>
#include "Math_Util.h"

namespace CrunchMath {

	float Radian(const float& thetha)
	{
		return ( thetha*(Pi / 180) );
	}

	float Degree(const float& radian)
	{
		return ( radian*(180 / Pi) );
	}

	float WrapPi(float& rad)
	{
		rad = rad + Pi;
		rad -= std::floor(rad / TwoPi) * TwoPi;
		rad -=  Pi;

		return rad;
	}

	float  acos2(float cosrad)
	{
		if (cosrad <= -1.0f)
			return Pi;

		else if (cosrad >= 1.0f)
			return 0.0f;


		return acos(cosrad);
	}

	float  asin2(float sinrad)
	{
		if (sinrad <= -1.0f)
			return -(PiOverTwo);

		else if (sinrad >= 1.0f)
			return (PiOverTwo);
		
		return  asin(sinrad);
	}
}