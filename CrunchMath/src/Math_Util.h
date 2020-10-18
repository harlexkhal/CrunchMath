#pragma once

    //******************************************************************
   //?? Before using any Function that involves you to use Degree/Angles, 
  //Convert to Radian First....with Esm::Radian(value);...Kapish
 //harlex Told ya so...  No jokes...its C++..lol.........................
//***********************************************************************
namespace CrunchMath {
	
	const float Pi = 3.141592654f;

	const float TwoPi = Pi * 2.0f;

	const float PiOverTwo = Pi / 2.0f;

	const float OneOverPi = 1.0f / Pi;

	const float OneOver2Pi = 1.0f / TwoPi;


	float Radian(const float& thetha);

	float Degree(const float& radian);

	float WrapPi(float& rad);

	float acos2(float cosrad);

	float asin2(float sinrad);
}