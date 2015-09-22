#ifndef VECTOR_H
#define VECTOR_H

#include "precision.h"

namespace Physics_Engine
{
	class Vector3
	{
	public:
		real x, y, z;

	private:
		real pad;

	public:
		Vector3()
			: x(0), y(0), z(0) { }

		Vector3(const real x, const real y, const real z)
			: x(x), y(y), z(z) {}

		real operator[](unsigned i) const
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		real& operator[](unsigned i)
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		void operator*=(const real value)
		{
			x *= value;
			y *= value;
			z *= value;
		}

		Vector3 operator*(const real value) const
		{
			return Vector3(x * value, y * value, z * value);
		}

		Vector3 operator/(const real value) const
		{
			return Vector3(x / value, y / value, x / value);
		}

		Vector3& Vector3::operator+=(const Vector3 &vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;

			return *this;
		}

		Vector3 operator + (const Vector3& vec) const
		{
			return Vector3(x + vec.x, y + vec.y, z + vec.z);
		}

		void operator -= (const Vector3& vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
		}

		Vector3 operator - (const Vector3& vec) const
		{
			return Vector3(x - vec.x, y - vec.y, z - vec.z);
		}

		void addScaledVector(const Vector3& vector, real scale)
		{
			x += vector.x * scale;
			y += vector.y * scale;
			z += vector.z * scale;
		}

		Vector3 componentProduct(const Vector3 &vector) const
		{
			return Vector3(x * vector.x, y * vector.y, z * vector.z);
		}

		void componentProductUpdate(const Vector3 &vector)
		{
			x *= vector.x;
			y *= vector.y;
			z *= vector.z;
		}

		real scalarProduct(const Vector3 &vector) const
		{
			return x*vector.x + y*vector.y + z*vector.z;
		}

		real operator * (const Vector3 &vector) const
		{
			return x*vector.x + y*vector.y + z*vector.z;
		}

		Vector3 vectorProduct(const Vector3 &vector) const
		{
			return Vector3(y*vector.z - z*vector.y,
				z*vector.x - x*vector.z,
				x*vector.y - y*vector.x);
		}

		// Use of % for cross product.
		Vector3 operator % (const Vector3 &vector) const		
		{
			return Vector3(y*vector.z - z*vector.y,
				z*vector.x - x*vector.z,
				x*vector.y - y*vector.x);
		}

		void makeOrthonormalBasis(Vector3 *a, Vector3 *b, Vector3 *c)
		{
			a->normalise();
			*c = *a % *b;	

			if (c->squareMagnitude() == 0.0)		
				return;		

			c->normalise();
			*b = *c % *a;
		}

		void clear()
		{
			x = y = z = 0;
		}

		void invert()
		{
			x = -x;
			y = -y;
			z = -z;
		}

		real magnitude() const
		{
			return real_sqrt(x*x + y*y + z*z);
		}

		real squareMagnitude() const
		{
			return x*x + y*y + z*z;
		}

		Vector3 normalise()
		{
			real length = magnitude();

			if (length > 0)
			{
				*this *= (real)1 / length;
			}

			return *this;
		}
	};
}
#endif