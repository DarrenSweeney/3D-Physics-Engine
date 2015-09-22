#include "pfGen.h"
#include <iostream>

using namespace std;

using namespace Physics_Engine;

void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();

	for (; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->particle, duration);
	}
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator *fg)
{
	ParticleForceRegistry::ParticleForceRegistration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ParticleGravity::updateForce(Particle* particle, real duration)
{
	if (!particle->hasFiniteMass())
		return;

	particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real _k1, real _k2)
: k1(_k1), k2(_k2)
{

}

void ParticleDrag::updateForce(Particle* particle, real duration)
{
	Vector3 force;
	particle->getVelocity(&force);

	// Calculate the total drag coeficient
	real dragCoefficient = force.magnitude();
	dragCoefficient = (k1 * dragCoefficient) + (k2 * (dragCoefficient * dragCoefficient));

	// Calculate the final force and apply it.
	force.normalise();
	force *= -dragCoefficient;
	particle->addForce(force);
}

ParticleSpring::ParticleSpring(Particle *particle, real spConstant, real rLength)
: other(particle), springConstant(spConstant), restLength(rLength)
{
	
}
 
void ParticleSpring::updateForce(Particle *particle, real duration)
{
	Vector3 springVector = particle->getPosition() - other->getPosition();
	real lenght = springVector.magnitude();
	real displacement = lenght - restLength;

	Vector3 restoringForce = springVector * (displacement * -springConstant);

	particle->addForce(restoringForce);
}

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 *anchor, real sConstant, real rLenght)
: anchor(anchor), springConstant(springConstant), restLength(rLenght)
{
	
}

void ParticleAnchoredSpring::updateForce(Particle *particle, real duration)
{
	Vector3 force;
	particle->getPosition(&force);
	force -= *anchor;

	real magnitude = force.magnitude();
	magnitude = (restLength - magnitude) * springConstant;

	force.normalise();
	force *= -magnitude;
	particle->addForce(force);
}

void ParticleBungee::updateForce(Particle *particle, real duration)
{
	Vector3 force;
	particle->getPosition(&force);
	force -= other->getPosition();

	real magnitude = force.magnitude();
	if (magnitude <= restLenght)
		return;

	magnitude = springConstant * (restLenght - magnitude);

	force.normalise();
	force *= -magnitude;
	particle->addForce(force);
}

void ParticleBuoyancy::updateForce(Particle *particle, real duration)
{
	real depth = particle->getPosition().x;

	if (depth >= waterHeight + maxDepth)
		return;

	Vector3 force(0, 0, 0);

	if (depth <= waterHeight - maxDepth)
	{
		force.y = liquidDensity * volume;
		particle->addForce(force);

		return;
	}

	force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
	particle->addForce(force);
}