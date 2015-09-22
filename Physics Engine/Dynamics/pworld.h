#ifndef PWORLD_H
#define PWORLD_H

#include "../Dynamics/plinks.h"
#include "../Dynamics/pfGen.h"

namespace Physics_Engine
{
	/*
		Keeps track of a set of particles and provides the
		means to update them all
	*/
	class ParticleWorld
	{
	public:
		typedef std::vector<Particle*> Particles;
		typedef std::vector<ParticleContactGenerator*> ContactGenerators;

	protected:
		// Holds the particles
		Particles particles;

		/*
			True if the world should calculate the number of iterations
			to give the contact resolver at each frame
		*/
		bool b_calculateIterations;

		// Holds the force generators for the particles in this world
		ParticleForceRegistry registry;

		// Holds the resolver for contacts
		ParticleContactResolver resolver;

		// Contact generators
		ContactGenerators contactGenerators;

		// Holds the list of contacts
		ParticleContact *contacts;

		// Holds the maximum number of contacts allowed (Size of the contacts array)
		unsigned maxContacts;

	public:
		/*
			Creates a new particle simulator that can handle up to the
			given number of contacts per frame. You can also optionally
			give a number of contact-resolution iterations to use. 
			If you don't give a number of iterations, then twice the 
			number of contacts will be used.
		*/
		ParticleWorld(unsigned maxContacts, unsigned iteration = 0);

		// Delects the simulator
		~ParticleWorld();			// ******** Look into desturctors more. How to clean up application ********

		/*
			Calls each of the registered contact generators to report
			their contacts. Returns the number of generated contacts.
		*/
		unsigned generateContacts();

		//	Integrates all the particles in the world by the given duration
		void intergrate(real duration);

		// Processes all the physics for the particle world
		void runPhysics(real duration);

		/*
			Initializes the world for a simulation frame. This clears
			the forces accumulators for particles in the world. After 
			calling this, the particles can have their forces for 
			this frame added
		*/
		void startFrame();

		// Returns the list of particles
		Particles& getParticles();

		// Returns the list of contact generators
		ContactGenerators& getContactGenerators();

		// Returns the force registry
		ParticleForceRegistry& getForceRegistry();
	};
}
#endif