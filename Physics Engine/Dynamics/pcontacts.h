#ifndef PCONTACTS_H
#define PCONTACTS_H

#include "particle.h"

namespace Physics_Engine
{
	class ParticleContact
	{
		/*
			The contact resolver object needs access into the contacts
			to set and effect the contact
		*/
		friend class ParticleContactResolver;

	public:
		Particle* particle[2];
		real restitution;
		Vector3 contactNormal;
		real penetration;
		Vector3 particleMovement[2];

	protected:
		void resolve(real duration);
		real calculateSeparatingVelocity() const;

	private:
		void resolveVelocity(real duration);
		void resolveInterpenetration(real duration);
	};

	class ParticleContactResolver
	{
	protected:
		unsigned iterations;
		unsigned iterationsUsed;

	public:
		ParticleContactResolver(unsigned iterations);
		void setIterations(unsigned iterations);
		void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration);
	};

	/*
		This is the basic polymorphic interface for contact generators
		applying to particles
	*/
	class ParticleContactGenerator
	{
	public:
		/*
			Fills the given contact structure with the generated contact.
			The contact pointer should point to the first avaiable contact 
			in a contact array, where limit is the maximum number of contacts
			in the array that can be written to. The method returns the number 
			of contacts that have been written
		*/
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
	};
}

#endif	// PCONTACTS_H