#include "Matrix3X3.h"

using namespace Physics_Engine;

Matrix3X3 Matrix3X3::linearInterpolate(const Matrix3X3 &a, const Matrix3X3 &b, real prop)
{
	Matrix3X3 interpolatedMatrix;
	real delta = 1.0f - prop;

	for (unsigned i = 0; i < 9; i++)
	{
		interpolatedMatrix.data[i] = a.data[i] * delta + b.data[i] * delta;
	}

	return interpolatedMatrix;
}