#include "plinks.h"

using namespace Physics_Engine;

real ParticleLink::currentLength() const
{
	Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();

	return relativePos.magnitude();
}

unsigned ParticleCable::addContact(ParticleContact *contact, unsigned limit) const
{
	real length = currentLength();

	if (length < maxLength)
	{
		return 0;
	}

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];

	Vector3 normal = contact->particle[1]->getPosition() - particle[0]->getPosition();
	normal.normalise();

	contact->contactNormal = normal;

	contact->penetration = length - maxLength;
	contact->restitution = restitution;

	return 1;
}

unsigned ParticleRod::addContact(ParticleContact *contact, unsigned limit) const
{
	real currentLen = currentLength();

	if (currentLen == length)
	{
		return 0;
	}

	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];

	Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
	normal.normalise();

	if (currentLen > length)
	{
		contact->contactNormal = normal;
		contact->penetration = currentLen - length;
	}
	else
	{
		contact->contactNormal = normal * -1;
		contact->penetration = length - currentLen;
	}
	contact->restitution = 0;

	return 1;
}

real ParticleConstraint::currentLength() const
{
	Vector3 relativePos = particle->getPosition() - anchor;
	return relativePos.magnitude();
}

unsigned ParticleCableConstraint::addContact(ParticleContact *contact, unsigned limit) const
{
	real lenght = currentLength();

	if (lenght < maxLength)
	{
		return 0;
	}
	contact->particle[0] = particle;
	contact->particle[1] = 0;

	Vector3 normal = anchor - particle->getPosition();
	normal.normalise();
	contact->contactNormal = normal;

	contact->penetration = lenght - maxLength;
	contact->restitution = restitution;
	
	return 1;
}

unsigned ParticleRodConstraint::addContact(ParticleContact *contact, unsigned limit) const
{
	real currentLen = currentLength();

	if (currentLen == lenght)
	{
		return 0;
	}

	contact->particle[0] = particle;
	contact->particle[1] = 0;

	Vector3 normal = anchor - particle->getPosition();
	normal.normalise();

	if (currentLen > lenght)
	{
		contact->contactNormal = normal;
		contact->penetration = lenght - currentLen;
	}
	else
	{
		contact->contactNormal = normal * -1;
		contact->penetration = currentLen - lenght;
	}

	contact->restitution = 0;

	return 1;
}