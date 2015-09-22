#ifndef PARTICLE_H
#define PARTICLE_H

#include "../Math/core.h"

namespace Physics_Engine
{
	class Particle
	{
	protected:
		Vector3 position;
		Vector3 oldPosition;	// For verlet integration.
		Vector3 velocity;
		Vector3 acceleration;
		Vector3 normal;
		real damping;
		real inverseMass;
		Vector3 forceAccum;

		bool movable;

	public:
		Particle();
		Particle(Vector3 &position);
		void intergrate(real duration);
		void verletIntegrate(float timeStep);
		void makeUnmovable();
		void offsetPos(const Vector3& v);
		void setMass(const real mass);
		real getMass() const;
		void setInverseMass(const real inverseMass);
		real getInverseMass() const;
		bool hasFiniteMass() const;
		Vector3& getNormal();
		void resetNormal(); 
		void setDamping(const real damping);
		real getDamping() const;
		void setPosition(const Vector3 &position);
		void setPosition(const real x, const real y, const real z);
		void getPosition(Vector3 *position) const;
		Vector3 getPosition() const;
		void setVelocity(const Vector3 &velocity);
		void setVelocity(const real x, const real y, const real z);
		void getVelocity(Vector3* vecloity) const;
		Vector3 getVelocity() const;
		void setAcceleration(const Vector3 &acceleration);
		void setAcceleration(const real x, const real y, const real z);
		void getAcceleration(Vector3 *acceleration) const;
		Vector3 getAcceleration() const;
		void addForce(const Vector3 &force);
		void clearAccumulator();
	};
}

#endif	// PARTICLE_H