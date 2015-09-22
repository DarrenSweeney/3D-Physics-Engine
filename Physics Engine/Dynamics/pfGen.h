#ifndef PFGEN_H
#define PFGEN_H

#include "particle.h"
#include <vector>

namespace Physics_Engine
{
	class ParticleForceGenerator
	{
	public:
		virtual void updateForce(Particle* particle, real duration) = 0;
	};

	class ParticleForceRegistry
	{
	protected:
		struct ParticleForceRegistration
		{
			Particle *particle;
			ParticleForceGenerator *fg;					
		};

		typedef std::vector<ParticleForceRegistration> Registry;
		Registry registrations;

	public:
		void add(Particle* particle, ParticleForceGenerator *fg);
		void remove(Particle* particle, ParticleForceGenerator *fg);
		void clear();
		void updateForces(real duration);
	};

	class ParticleGravity : ParticleForceGenerator
	{
		Vector3 gravity;

	public:
		ParticleGravity(const Vector3 &gravity);

		virtual void updateForce(Particle* particle, real duration);
	};

	class ParticleDrag : public ParticleForceGenerator
	{
		real k1;
		real k2;

	public:
		ParticleDrag(real k1, real k2);
		virtual void updateForce(Particle* particle, real duration);
	};

	class ParticleSpring : public ParticleForceGenerator
	{
		Particle *other;
		real springConstant;
		real restLength;

	public:
		ParticleSpring(Particle *other, real springConstant, real restLength);
		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleAnchoredSpring : public ParticleForceGenerator
	{
	protected:
		Vector3 *anchor;
		real springConstant;
		real restLength;

	public:
		ParticleAnchoredSpring(Vector3 *anchor, real springConstant, real restLenght);
		virtual void updateForce(Particle *partilce, real duration);
	};

	class ParticleBungee : public ParticleForceGenerator
	{
		Particle *other;
		real springConstant;
		real restLenght;

	public:
		ParticleBungee(Particle* other, real springConstant, real restLenght);
		virtual void updateForce(Particle *particle, real duration);
	};

	class ParticleBuoyancy : public ParticleForceGenerator
	{
		real maxDepth;
		real volume;
		real waterHeight;
		real liquidDensity;

	public:
		ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
		virtual void updateForce(Particle *particle, real duration);
	};
}

#endif