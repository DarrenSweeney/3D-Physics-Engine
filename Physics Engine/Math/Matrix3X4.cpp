#include "Matrix3X4.h"

using namespace Physics_Engine;

real Matrix3X4::getDeterminate() const
{
	return data[8] * data[5] * data[2] +
		data[4] * data[9] * data[2] +
		data[8] * data[1] * data[6] -
		data[0] * data[9] * data[6] -
		data[4] * data[1] * data[10] +
		data[0] * data[5] * data[10];
}

void Matrix3X4::setInverse(const Matrix3X4 &matrix)
{
	// Make sure the determinate is non-zero
	real det = getDeterminate();

	if (det == 0)
		return;

	det = ((real)1.0f) / det;

	data[0] = (-matrix.data[9] * matrix.data[6] + matrix.data[5] * matrix.data[10])*det;
	data[4] = (matrix.data[8] * matrix.data[6] - matrix.data[4] * matrix.data[10])*det;
	data[8] = (-matrix.data[8] * matrix.data[5] + matrix.data[4] * matrix.data[9])*det;

	data[1] = (matrix.data[9] * matrix.data[2] - matrix.data[1] * matrix.data[10])*det;
	data[5] = (-matrix.data[8] * matrix.data[2] + matrix.data[0] * matrix.data[10])*det;
	data[9] = (matrix.data[8] * matrix.data[1] - matrix.data[0] * matrix.data[9])*det;

	data[2] = (-matrix.data[5] * matrix.data[2] + matrix.data[1] * matrix.data[6])*det;
	data[6] = (+matrix.data[4] * matrix.data[2] - matrix.data[0] * matrix.data[6])*det;
	data[10] = (-matrix.data[4] * matrix.data[1] + matrix.data[0] * matrix.data[5])*det;

	data[3] = (matrix.data[9] * matrix.data[6] * matrix.data[3]
		- matrix.data[5] * matrix.data[10] * matrix.data[3]
		- matrix.data[9] * matrix.data[2] * matrix.data[7]
		+ matrix.data[1] * matrix.data[10] * matrix.data[7]
		+ matrix.data[5] * matrix.data[2] * matrix.data[11]
		- matrix.data[1] * matrix.data[6] * matrix.data[11])*det;
	data[7] = (-matrix.data[8] * matrix.data[6] * matrix.data[3]
		+ matrix.data[4] * matrix.data[10] * matrix.data[3]
		+ matrix.data[8] * matrix.data[2] * matrix.data[7]
		- matrix.data[0] * matrix.data[10] * matrix.data[7]
		- matrix.data[4] * matrix.data[2] * matrix.data[11]
		+ matrix.data[0] * matrix.data[6] * matrix.data[11])*det;
	data[11] = (matrix.data[8] * matrix.data[5] * matrix.data[3]
		- matrix.data[4] * matrix.data[9] * matrix.data[3]
		- matrix.data[8] * matrix.data[1] * matrix.data[7]
		+ matrix.data[0] * matrix.data[9] * matrix.data[7]
		+ matrix.data[4] * matrix.data[1] * matrix.data[11]
		- matrix.data[0] * matrix.data[5] * matrix.data[11])*det;
}