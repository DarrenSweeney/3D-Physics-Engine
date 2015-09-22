#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#include "particle.h"
#include "../Math/core.h"
#include "../Math/precision.h"

namespace Physics_Engine
{
	class Constraint
	{
	private:
		real rest_distance;

	public:
		Particle *p1, *p2;

		Constraint(Particle *p1, Particle *p2)
			: p1(p1), p2(p2)
		{
			rest_distance = (p1->getPosition() - p2->getPosition()).magnitude();
		}

		void satisfyConstraint()
		{
			Vector3 delta = p2->getPosition() - p1->getPosition();
			real deltalendth = delta.magnitude();
			real diff = (deltalendth - rest_distance) / deltalendth;

			delta *= 0.5f * diff;

			p1->offsetPos(delta);
			p2->offsetPos(Vector3(-delta.x, -delta.y, -delta.z));
		}
	};
}
#endif