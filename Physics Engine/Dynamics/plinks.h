#ifndef PLINKS_H
#define PLINKS_H

#include "pcontacts.h"

namespace Physics_Engine
{
	class ParticleLink : public ParticleContactGenerator
	{
	public:
		Particle* particle[2];

	protected:
		real currentLength() const;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
	};

	class ParticleCable : public ParticleLink
	{
	public:
		real maxLength;
		real restitution;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
	};

	class ParticleRod : public ParticleLink
	{
	public:
		real length;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
	};

	class ParticleConstraint : public ParticleContactGenerator
	{
	public:
		Particle *particle;
		Vector3 anchor;

	protected:
		real currentLength() const;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const = 0;
	};

	class ParticleCableConstraint : public ParticleConstraint
	{
	public:
		real maxLength;
		real restitution;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
	};

	class ParticleRodConstraint : public ParticleConstraint
	{
	public:
		real lenght;

	public:
		virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
	};
}

#endif