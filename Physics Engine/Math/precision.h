#include "math.h"
#include <float.h>

namespace Physics_Engine
{
	/*
		Define a real number precsion. This physics engine can be compiled in
		single or double precision versions. By default, single
		precision is provided
	*/
	typedef double real;

#define real_sqrt sqrt
#define real_pow pow	
#define REAL_MAX DBL_MAX
#define real_abs fabsf
#define real_fmod fmod
#define R_PI 3.14159265358979
}