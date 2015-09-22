#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include "precision.h"
#include "Vector3.h"
#include "Quaternion.h"

namespace Physics_Engine
{
	class Matrix3X3
	{
	public:
		real data[9];

		Matrix3X3()
		{
			data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
				data[6] = data[7] = data[8] = 0;
		}

		Matrix3X3(real c0, real c1, real c2, real c3, real c4, real c5,
			real c6, real c7, real c8)
		{
			data[0] = c0; data[1] = c1; data[2] = c2;
			data[3] = c3; data[4] = c4; data[5] = c5;
			data[6] = c6; data[7] = c7; data[8] = c8;
		}

		void setColumns(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree)
		{
			data[0] = compOne.x; data[1] = compTwo.x; data[2] = compThree.x;
			data[3] = compOne.y; data[4] = compTwo.y; data[5] = compThree.y;
			data[6] = compOne.z; data[7] = compTwo.z; data[8] = compThree.z;
		}

		void setInverse(const Matrix3X3 &matrix)
		{
			real t1 = matrix.data[0] * matrix.data[4];
			real t2 = matrix.data[0] * matrix.data[5];
			real t3 = matrix.data[1] * matrix.data[3];
			real t4 = matrix.data[2] * matrix.data[3];
			real t5 = matrix.data[1] * matrix.data[6];
			real t6 = matrix.data[2] * matrix.data[6];

			// Calculate the determinate.
			real det = (t1 * matrix.data[8] - t2 * matrix.data[7] - t3 * matrix.data[8] +
				t4 * matrix.data[7] + t5 * matrix.data[5] - t6 * matrix.data[4]);

			// Make sure the determinant is not zero
			if (det == (real)0.0f)
				return;

			real invd = (real)1.0f / det;

			data[0] = (matrix.data[4] * matrix.data[8] - matrix.data[5] * matrix.data[7]) * invd;
			data[1] = -(matrix.data[1] * matrix.data[8] - matrix.data[2] * matrix.data[7]) * invd;
			data[2] = (matrix.data[1] * matrix.data[5] - matrix.data[2] * matrix.data[4]) * invd;
			data[3] = -(matrix.data[3] * matrix.data[8] - matrix.data[5] * matrix.data[6]) * invd;
			data[4] = (matrix.data[0] * matrix.data[8] - t6) * invd;
			data[5] = -(t2 - t4) * invd;
			data[6] = (matrix.data[3] * matrix.data[7] - matrix.data[4] * matrix.data[6]) * invd;
			data[7] = -(matrix.data[0] * matrix.data[7] - t5) * invd;
			data[8] = (t1 - t3) * invd;
		}

		Matrix3X3 inverse() const
		{
			Matrix3X3 result;
			result.setInverse(*this);

			return result;
		}

		void invert()
		{
			setInverse(*this);
		}

		void setInertiaTensorCoeffs(real ix, real iy, real iz,
			real ixy = 0, real ixz = 0, real iyz = 0)
		{
			data[0] = ix;	data[1] = -ixy;	data[2] = -ixz;
			data[3] = -ixy;	data[4] = iy;	data[5] = -iyz;
			data[6] = -ixz;	data[7] = -iyz;	data[8] = iz;
		}

		void setBlockInertiaTensor(const Vector3 &halfSizes, real mass)
		{
			Vector3 squares = halfSizes.componentProduct(halfSizes);
			setInertiaTensorCoeffs(0.3f*mass*(squares.y + squares.z),
				0.3f*mass*(squares.x + squares.z),
				0.3f*mass*(squares.x + squares.y));
		}

		void setSkewSymmetric(const Vector3 &vector)
		{
			data[0] = data[4] = data[8] = 0;
			data[1] = -vector.z;
			data[2] = vector.y;
			data[3] = vector.z;
			data[5] = -vector.x;
			data[6] = -vector.y;
			data[7] = vector.x;
		}

		void setTranspose(const Matrix3X3 &matrix)
		{
			data[0] = matrix.data[0];
			data[1] = matrix.data[3];
			data[2] = matrix.data[6];
			data[3] = matrix.data[1];
			data[4] = matrix.data[4];
			data[5] = matrix.data[7];
			data[6] = matrix.data[2];
			data[7] = matrix.data[5];
			data[8] = matrix.data[8];
		}

		Matrix3X3 transpose() const
		{
			Matrix3X3 result;
			result.setInverse(*this);

			return result;
		}

		void setOrientation(const Quaternion &q)
		{
			data[0] = 1 - (2 * q.j*q.j + 2 * q.k*q.k);
			data[1] = 2 * q.i*q.j + 2 * q.k*q.r;
			data[2] = 2 * q.i*q.k - 2 * q.j*q.r;
			data[3] = 2 * q.i*q.j - 2 * q.k*q.r;
			data[4] = 1 - (2 * q.i*q.i + 2 * q.k*q.k);
			data[5] = 2 * q.j*q.k + 2 * q.i*q.r;
			data[6] = 2 * q.i*q.k + 2 * q.j*q.r;
			data[7] = 2 * q.j*q.k - 2 * q.i*q.r;
			data[8] = 1 - (2 * q.i*q.i + 2 * q.j*q.j);
		}

		static Matrix3X3 linearInterpolate(const Matrix3X3 &a, const Matrix3X3 &b, real prop);

		Vector3 operator*(const Vector3 &vector) const
		{
			return Vector3(
				vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
				vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
				vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
				);
		}

		Vector3 transform(const Vector3 &vector) const
		{
			return (*this) * vector;
		}

		Vector3 transformTranspose(const Vector3 &vector) const
		{
			return Vector3(vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
				vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
				vector.x * data[2] + vector.y * data[5] + vector.z * data[8]);
		}

		Matrix3X3 operator*(const Matrix3X3 &otherMatrix) const
		{
			return Matrix3X3(data[0] * otherMatrix.data[0] + data[1] * otherMatrix.data[3] + data[2] * otherMatrix.data[6],
				data[0] * otherMatrix.data[1] + data[1] * otherMatrix.data[4] + data[2] * otherMatrix.data[7],
				data[0] * otherMatrix.data[2] + data[1] * otherMatrix.data[5] + data[2] * otherMatrix.data[8],

				data[3] * otherMatrix.data[0] + data[4] * otherMatrix.data[3] + data[5] * otherMatrix.data[6],
				data[3] * otherMatrix.data[1] + data[4] * otherMatrix.data[4] + data[5] * otherMatrix.data[7],
				data[3] * otherMatrix.data[2] + data[4] * otherMatrix.data[5] + data[5] * otherMatrix.data[8],

				data[6] * otherMatrix.data[0] + data[7] * otherMatrix.data[3] + data[8] * otherMatrix.data[6],
				data[6] * otherMatrix.data[1] + data[7] * otherMatrix.data[4] + data[8] * otherMatrix.data[7],
				data[6] * otherMatrix.data[2] + data[7] * otherMatrix.data[5] + data[8] * otherMatrix.data[8]);
		}

		void operator*=(const Matrix3X3 &otherMatrix)
		{
			real t1;
			real t2;
			real t3;

			t1 = data[0] * otherMatrix.data[0] + data[1] * otherMatrix.data[3] + data[2] * otherMatrix.data[6];
			t2 = data[0] * otherMatrix.data[1] + data[1] * otherMatrix.data[4] + data[2] * otherMatrix.data[7];
			t3 = data[0] * otherMatrix.data[2] + data[1] * otherMatrix.data[5] + data[2] * otherMatrix.data[8];
			data[0] = t1;
			data[1] = t2;
			data[2] = t3;

			t1 = data[3] * otherMatrix.data[0] + data[4] * otherMatrix.data[3] + data[5] * otherMatrix.data[6];
			t2 = data[3] * otherMatrix.data[1] + data[4] * otherMatrix.data[4] + data[5] * otherMatrix.data[7];
			t3 = data[3] * otherMatrix.data[2] + data[4] * otherMatrix.data[5] + data[5] * otherMatrix.data[8];
			data[3] = t1;
			data[4] = t2;
			data[5] = t3;

			t1 = data[6] * otherMatrix.data[0] + data[7] * otherMatrix.data[3] + data[8] * otherMatrix.data[6];
			t2 = data[7] * otherMatrix.data[1] + data[7] * otherMatrix.data[4] + data[8] * otherMatrix.data[7];
			t3 = data[8] * otherMatrix.data[2] + data[7] * otherMatrix.data[5] + data[8] * otherMatrix.data[8];
			data[6] = t1;
			data[7] = t2;
			data[8] = t3;
		}

		void operator*=(const real scalar)
		{
			data[0] *= scalar;	data[1] *= scalar;	data[2] *= scalar;
			data[3] *= scalar;	data[4] *= scalar;	data[5] *= scalar;
			data[6] *= scalar;	data[7] *= scalar;	data[8] *= scalar;
		}

		void operator+=(const Matrix3X3 &o)
		{
			data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
			data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
			data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
		}
	};
}
#endif