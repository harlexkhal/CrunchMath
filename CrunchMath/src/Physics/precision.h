#pragma once
#include <float.h>

namespace CrunchPhysx {

#if 1
    //Single Precision Mode
    typedef float cpfloat;
    #define cp_MAX FLT_MAX
    #define cp_sqrt sqrtf
    #define cp_abs fabsf
    #define cp_sin sinf
    #define cp_cos cosf
    #define cp_exp expf
    #define cp_pow powf
    #define cp_fmod fmodf
    #define cp_epsilon FLT_EPSILON
    #define R_PI 3.14159265358979f

#else
    // Dounble Precision Mode
    #define DOUBLE_PRECISION
    typedef double cpfloat;
    #define cp_MAX DBL_MAX
    #define cp_sqrt sqrt
    #define cp_abs fabs
    #define cp_sin sin
    #define cp_cos cos
    #define cp_exp exp
    #define cp_pow pow
    #define cp_fmod fmod
    #define cp_epsilon DBL_EPSILON
    #define R_PI 3.14159265358979
#endif
}