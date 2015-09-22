#include <windows.h> 
#include "cloth.h"
#include <stdio.h>
#include <vector>
#include <string>
#include "particle.h"
#include "../Math/core.h"
#include "constraint.h"
#include <GL/gl.h>
#include <iostream>
#include "../DebugRender/DebugDrawManager.h"

using namespace Physics_Engine;

Cloth::Cloth(real width, real height, int num_particles_width, int num_particles_height)
: num_particles_width(num_particles_width), num_particles_height(num_particles_height)
{
	particles.resize(num_particles_width*num_particles_height);

	for (int x = 0; x < num_particles_width; x++)
	{
		for (int y = 0; y < num_particles_height; y++)
		{
			Vector3 pos = Vector3(width * (x / (real)num_particles_width),
				0,
				height * (y / (real)num_particles_height));

			particles[y*num_particles_width + x] = Particle(pos);
		}
	}

	/*
	(x,y)   *--* (x+1,y)
	|  |
	|  |
	(x,y+1) *--* (x+1,y+1)
	*/

	for (int x = 0; x < num_particles_width; x++)
	{
		for (int y = 0; y < num_particles_height; y++)
		{
			if (x < num_particles_width - 1)
				makeConstraint(getParticle(x, y), getParticle(x + 1, y)); // (x,y) --- (x+1,y)

			if (y < num_particles_height - 1)
				makeConstraint(getParticle(x, y), getParticle(x, y +1)); // (x,y) --- (x,y+1)

			if (x < num_particles_width - 1 && y < num_particles_height - 1)
			{
				makeConstraint(getParticle(x + 1, y), getParticle(x + 1, y + 1)); // (x+1,y) --- (x,y+1)
				makeConstraint(getParticle(x, y + 1), getParticle(x + 1, y + 1)); // (x,y+1) --- (x+1,y+1)
			}

			// Shear Constraints.
			if (x<num_particles_width - 1 && y<num_particles_height - 1)
				makeConstraint(getParticle(x, y), getParticle(x + 1, y + 1));
			if (x<num_particles_width - 1 && y<num_particles_height - 1)
				makeConstraint(getParticle(x + 1, y), getParticle(x, y + 1));

			// Bend Constraints.
			if (x<num_particles_width - 2 && y<num_particles_height - 2)
				makeConstraint(getParticle(x, y), getParticle(x + 2, y + 2));
			if (x<num_particles_width - 2 && y<num_particles_height - 2)
				makeConstraint(getParticle(x + 2, y), getParticle(x, y + 2));
		}
	}

	// Constrain 2 point on the cloth.
	getParticle(0, 0)->makeUnmovable();
	getParticle(num_particles_width - 1, 0)->makeUnmovable();

}

Particle* Cloth::getParticle(int x, int y)
{
	return &particles[y*num_particles_width + x];
}

void Cloth::makeConstraint(Particle *p1, Particle *p2)
{
	constraints.push_back(Constraint(p1, p2));
}

Vector3 Cloth::calcTriangleNormal(Particle *p1, Particle *p2, Particle *p3)
{
	Vector3 pos1 = p1->getPosition();
	Vector3 pos2 = p2->getPosition();
	Vector3 pos3 = p3->getPosition();

	Vector3 v1 = pos2 - pos1;
	Vector3 v2 = pos3 - pos1;

	Vector3 norm = v1.vectorProduct(v2);

	return norm;
}

void Cloth::addWindToTriangles(Particle *p1, Particle *p2, Particle *p3, const Vector3 &direction)
{
	Vector3 normal = calcTriangleNormal(p1, p2, p2);

	Vector3 d = normal.normalise();
	Vector3 force = normal*(d.scalarProduct(direction));

	p1->addForce(force);
	p2->addForce(force);
	p3->addForce(force);
}

void Cloth::drawTriangle(Particle *p1, Particle *p2, Particle *p3, const Vector3& color)
{
	glColor3d(color.x, color.y, color.z);
	glVertex3d(p1->getPosition().x, p1->getPosition().y, p1->getPosition().z);
	glVertex3d(p2->getPosition().x, p2->getPosition().y, p2->getPosition().z);
	glVertex3d(p3->getPosition().x, p3->getPosition().y, p3->getPosition().z);
}

void Cloth::debugDraw()
{
	g_debugDrawManager.AddCross(Vector3(getParticle(0, 0)->getPosition()), Vector3(0.0f, 0.5f, 0.0f), 0.5f);
	g_debugDrawManager.AddCross(Vector3(getParticle(num_particles_width - 1, 0)->getPosition()), Vector3(0.0f, 0.5f, 0.0f), 0.5f);

	for (int x = 0; x < num_particles_width; x++)
	{
		for (int y = 0; y < num_particles_height; y++)
		{
			g_debugDrawManager.AddLine(getParticle(x, y)->getPosition(), getParticle(x, y)->getNormal() + getParticle(x, y)->getPosition(), Vector3(1, 0, 0), 0.04f);
		}
	}
}

void Cloth::draw()
{
	parts particle;
	for (particle = particles.begin(); particle != particles.end(); particle++)
	{
		(*particle).resetNormal();
	}

	for (int x = 0; x < num_particles_width - 1; x++)
	{
		for (int y = 0; y < num_particles_height - 1; y++)
		{
			calcTriangleNormal(getParticle(x + 1, y), getParticle(x, y), getParticle(x, y + 1));
			calcTriangleNormal(getParticle(x + 1, y + 1), getParticle(x + 1, y), getParticle(x, y + 1));
		}
	}

	glBegin(GL_TRIANGLES);
	for (int x = 0; x < num_particles_width - 1; x++)
	{
		for (int y = 0; y < num_particles_height - 1; y++)
		{
			Vector3 color(0.0, 0.0, 0.0);
			if (x % 2)
				color = Vector3(0.3f, 0.3f, 0.3f);
			else
				color = Vector3(0.2f, 0.4f, 0.9f);

			drawTriangle(getParticle(x + 1, y), getParticle(x, y), getParticle(x, y + 1), color);
			drawTriangle(getParticle(x + 1, y + 1), getParticle(x + 1, y), getParticle(x, y + 1), color);
		}
	}
	glEnd();
}

void Cloth::timeStep(real duration)
{
	constr constraint;
	for (int i = 0; i<CONSTRAINT_ITERATIONS; i++)
	{
		for (constraint = constraints.begin(); constraint != constraints.end(); constraint++)
		{
			(*constraint).satisfyConstraint();
		}
	}

	parts particle;
	for (particle = particles.begin(); particle != particles.end(); particle++)
	{
		//(*particle).intergrate(duration);
		(*particle).verletIntegrate(0.5);
	}
}

void Cloth::addForce(const Vector3 &direction)
{
	parts particle;
	for (particle = particles.begin(); particle != particles.end(); particle++)
	{
		(*particle).addForce(direction * 0.5 * 0.5);
	}

}

void Cloth::addWindForce(const Vector3 direction)
{
	for (int x = 0; x < num_particles_width - 1; x++)
	{
		for (int y = 0; y < num_particles_height - 1; y++)
		{
			addWindToTriangles(getParticle(x + 1, y), getParticle(x, y), getParticle(x, y + 1), direction);
			addWindToTriangles(getParticle(x + 1, y + 1), getParticle(x + 1, y), getParticle(x, y + 1), direction);
		}
	}
}

void Cloth::ballCollision(const Sphere& sphere)
{
	parts particle;
	for (particle = particles.begin(); particle != particles.end(); particle++)
	{
		Vector3 v = (*particle).getPosition() - sphere.pos;

		real l = v.magnitude();

		if (v.magnitude() < sphere.radius)
		{
			(*particle).offsetPos(v.normalise()*(sphere.radius - l));
		}
	}
}