#ifndef MATRIX3X4_H
#define MATRIX3X4_H

#include "precision.h"
#include "Quaternion.h"
#include "Vector3.h"

namespace Physics_Engine
{
	class Matrix3X4
	{
	public:
		real data[12];

		Matrix3X4()
		{
			data[1] = data[2] = data[3] = data[4] = data[6] =
				data[7] = data[8] = data[9] = data[11] = 0;
			data[0] = data[5] = data[10] = 1;
		}

		real getDeterminate() const;
		void setInverse(const Matrix3X4 &matrix);

		Matrix3X4 Inverse() const
		{
			Matrix3X4 result;
			result.setInverse(*this);

			return result;
		}

		void invert()
		{
			setInverse(*this);
		}

		void setOrientationAndPos(const Quaternion &q, const Vector3 &pos)
		{
			/*
			0,  1,  2,  3,
			4,  5,  6,  7,
			8,  9, 10, 11,
			12, 13, 14, 15 // (0, 0, 0, 1)
			*/
			data[0] = 1 - (2 * q.j*q.j + 2 * q.k*q.k);
			data[1] = 2 * q.i*q.j + 2 * q.k*q.r;
			data[2] = 2 * q.i*q.k - 2 * q.j*q.r;
			data[3] = pos.x;

			data[4] = 2 * q.i*q.j - 2 * q.k*q.r;
			data[5] = 1 - (2 * q.i*q.i + 2 * q.k*q.k);
			data[6] = 2 * q.j*q.k + 2 * q.i*q.r;
			data[7] = pos.y;

			data[8] = 2 * q.i*q.k + 2 * q.j*q.r;
			data[9] = 2 * q.j*q.k - 2 * q.i*q.r;
			data[10] = 1 - (2 * q.i*q.i + 2 * q.j*q.j);
			data[11] = pos.z;
		}

		Vector3 getAxisVector(unsigned i) const
		{
			return Vector3(data[i], data[i + 4], data[i + 8]);
		}

		void fillGLArray(float array[16]) const
		{
			array[0] = (float)data[0];
			array[1] = (float)data[4];
			array[2] = (float)data[8];
			array[3] = (float)0;

			array[4] = (float)data[1];
			array[5] = (float)data[5];
			array[6] = (float)data[9];
			array[7] = (float)0;

			array[8] = (float)data[2];
			array[9] = (float)data[6];
			array[10] = (float)data[10];
			array[11] = (float)0;

			array[12] = (float)data[3];
			array[13] = (float)data[7];
			array[14] = (float)data[11];
			array[15] = (float)1;
		}

		Vector3 transformInverse(const Vector3 &vector) const
		{
			Vector3 tmp = vector;
			tmp.x -= data[3];
			tmp.y -= data[7];
			tmp.z -= data[11];

			return Vector3(
				tmp.x * data[0] +
				tmp.y * data[4] +
				tmp.z * data[8],

				tmp.x * data[1] +
				tmp.y * data[5] +
				tmp.z * data[9],

				tmp.x * data[2] +
				tmp.y * data[6] +
				tmp.z * data[10]);
		}

		Vector3 transformDirection(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] +
				vector.y * data[1] +
				vector.z * data[2],

				vector.x * data[4] +
				vector.y * data[5] +
				vector.z * data[6],

				vector.x * data[8] +
				vector.y * data[9] +
				vector.z * data[10]);
		}

		Vector3 transformInverseDirection(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] +
				vector.y * data[4] +
				vector.z * data[8],

				vector.x * data[1] +
				vector.y * data[5] +
				vector.z * data[9],

				vector.x * data[2] +
				vector.y * data[6] +
				vector.z * data[10]
				);
		}

		Vector3 operator*(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] +
				vector.y * data[1] +
				vector.z * data[2] + data[3],

				vector.x * data[4] +
				vector.y * data[5] +
				vector.z * data[6] + data[7],

				vector.x * data[8] +
				vector.y * data[9] +
				vector.z * data[10] + data[11]
				);
		}

		Vector3 transform(const Vector3 &vector) const
		{
			return (*this) * vector;
		}

		Matrix3X4 operator*(const Matrix3X4 &otherMatrix) const
		{
			Matrix3X4 result;
			result.data[0] = otherMatrix.data[0] * data[0] + otherMatrix.data[4] * data[1] + otherMatrix.data[8] * data[2];
			result.data[4] = otherMatrix.data[0] * data[4] + otherMatrix.data[4] * data[5] + otherMatrix.data[8] * data[6];
			result.data[8] = otherMatrix.data[0] * data[8] + otherMatrix.data[4] * data[9] + otherMatrix.data[8] * data[10];

			result.data[1] = otherMatrix.data[1] * data[0] + otherMatrix.data[5] * data[1] + otherMatrix.data[9] * data[2];
			result.data[5] = otherMatrix.data[1] * data[4] + otherMatrix.data[5] * data[5] + otherMatrix.data[9] * data[6];
			result.data[9] = otherMatrix.data[1] * data[8] + otherMatrix.data[5] * data[9] + otherMatrix.data[9] * data[10];

			result.data[2] = otherMatrix.data[2] * data[0] + otherMatrix.data[6] * data[1] + otherMatrix.data[10] * data[2];
			result.data[6] = otherMatrix.data[2] * data[4] + otherMatrix.data[6] * data[5] + otherMatrix.data[10] * data[6];
			result.data[10] = otherMatrix.data[2] * data[8] + otherMatrix.data[6] * data[9] + otherMatrix.data[10] * data[10];

			result.data[3] = otherMatrix.data[3] * data[0] + otherMatrix.data[7] * data[1] + (otherMatrix.data[11] * data[2]) + data[3];
			result.data[7] = otherMatrix.data[3] * data[4] + otherMatrix.data[7] * data[5] + (otherMatrix.data[11] * data[6]) + data[7];
			result.data[11] = otherMatrix.data[3] * data[8] + otherMatrix.data[7] * data[9] + (otherMatrix.data[11] * data[10]) + data[11];

			return result;
		}
	};
}
#endif