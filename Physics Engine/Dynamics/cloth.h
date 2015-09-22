#ifndef __Cloth_h
#define __Cloth_h

#include <vector>
#include <string>
#include "particle.h"
#include "../Math/core.h"
#include "constraint.h"

namespace Physics_Engine
{
#define CONSTRAINT_ITERATIONS 5

	class Sphere
	{
	public:
		Vector3 pos;
		real radius;

	public:
		Sphere(Vector3 p, real r)
		{
			pos = p;
			radius = r;
		}
	};

	class Cloth
	{
	private:

		typedef std::vector<Particle>::iterator parts;
		typedef std::vector<Constraint>::iterator constr;
		int num_particles_width;
		int num_particles_height;
		std::vector<Particle> particles;
		std::vector<Constraint> constraints;

		Particle* getParticle(int x, int y);
		void makeConstraint(Particle *p1, Particle *p2);
		Vector3 calcTriangleNormal(Particle *p1, Particle *p2, Particle *p3);
		void drawTriangle(Particle *p1, Particle *p2, Particle *p3, const Vector3& color);

	public:

		Cloth(real width, real height, int num_particles_width, int num_particles_height);
		void draw();
		void timeStep(real duration);
		void addForce(const Vector3& direction);
		void addWindToTriangles(Particle *p1, Particle *p2, Particle *p3, const Vector3 &direction);
		void addWindForce(const Vector3 direction);
		void ballCollision(const Sphere& sphere);
		void debugDraw();
	};
}
#endif