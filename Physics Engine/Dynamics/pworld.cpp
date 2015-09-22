#include "pworld.h"

using namespace Physics_Engine;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
: resolver(iterations), maxContacts(maxContacts)
{
	contacts = new ParticleContact[maxContacts];
	b_calculateIterations = (iterations == 0);
}

ParticleWorld::~ParticleWorld()
{
	delete[] contacts;
}

void ParticleWorld::startFrame()
{
	for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
	{
		// Removes all the forces from the accumulator
		(*p)->clearAccumulator();
	}
}

unsigned ParticleWorld::generateContacts()
{
	unsigned limit = maxContacts;
	ParticleContact *nextContact = contacts;

	for (ContactGenerators::iterator g = contactGenerators.begin();
		g != contactGenerators.end(); g++)
	{
		unsigned used = (*g)->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		/*
			We have run out of contacts to fill. This means we are
			missing contacts
		*/
		if (limit <= 0)
			break;
	}

	// Return the number of contacts used
	return maxContacts - limit;
}

void ParticleWorld::intergrate(real duration)
{
	for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
	{
		// Intergrate the particle by the given duration
		(*p)->intergrate(duration);
	}
}

void ParticleWorld::runPhysics(real duration)
{
	// First apply the force generators
	registry.updateForces(duration);

	// Then we integrate the objects
	intergrate(duration);

	// Generate contacts
	unsigned usedContacts = generateContacts();

	if (usedContacts)
	{
		if (b_calculateIterations)
			resolver.setIterations(usedContacts * 2);

		resolver.resolveContacts(contacts, usedContacts, duration);
	}
}

// List of particles ... Method to implement
ParticleWorld::Particles& ParticleWorld::getParticles()
{
	return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators()
{
	return contactGenerators;
}

ParticleForceRegistry& ParticleWorld::getForceRegistry()
{
	return registry;
}