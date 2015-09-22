#include "pcontacts.h"

using namespace Physics_Engine;

void ParticleContact::resolve(real duration)
{
	resolveVelocity(duration);
	resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
	Vector3 relativeVelocity = particle[0]->getVelocity();

	if (particle[1])
		relativeVelocity -= particle[1]->getVelocity();

	return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration)
{
	real seperatingVelocity = calculateSeparatingVelocity();

	if (seperatingVelocity > 0)
	{
		/* 
		   The contact is either sepearting or stationary
		   no impluse is required.
		*/
		return;
	}

	real newSepVelocity = -seperatingVelocity * restitution;

	Vector3 accCausedVelocity = particle[0]->getAcceleration();
	if (particle[1])
		accCausedVelocity -= particle[1]->getAcceleration();

	real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

	if (accCausedSepVelocity < 0)
	{
		newSepVelocity += restitution * accCausedSepVelocity;

		if (newSepVelocity < 0)
			newSepVelocity = 0;
	}

	real deltaVelocity = newSepVelocity - seperatingVelocity;

	real totalInverseMass = particle[0]->getInverseMass();
	if (particle[1])
		totalInverseMass += particle[1]->getInverseMass();

	// If all particles have infinite mass, then impluses have no effect.
	if (totalInverseMass <= 0)
		return;

	real impulse = deltaVelocity / totalInverseMass;

	Vector3 impulsePerInversMass = contactNormal * impulse;

	particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerInversMass * particle[0]->getInverseMass());

	if (particle[1])
	{
		particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerInversMass * -particle[1]->getInverseMass());
	}
}

void ParticleContact::resolveInterpenetration(real duration)
{
	if (penetration <= 0)
		return;

	real totalInverseMass = particle[0]->getInverseMass();
	if (particle[1])
		totalInverseMass += particle[1]->getInverseMass();

	if (totalInverseMass <= 0)
		return;

	Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

	particleMovement[0] = movePerIMass * particle[0]->getInverseMass();

	if (particle[1])
		particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
	else
		particleMovement[1].clear();

	particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);

	if (particle[1])
		particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
{
	ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
	ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, unsigned numContacts, real duration)
{
	unsigned i;

	iterationsUsed = 0;
	while (iterationsUsed < iterations)
	{
		real max = REAL_MAX;
		unsigned maxIndex = numContacts;
		for (int i = 0; i < numContacts; i++)
		{
			real sepVelocity = contactArray[i].calculateSeparatingVelocity();

			if (sepVelocity < max && 
				(sepVelocity < 0 || contactArray[i].penetration > 0))
			{
				max = sepVelocity;
				maxIndex = i;
			}
		}

		if (maxIndex == numContacts)
			break;

		contactArray[maxIndex].resolve(duration);

		Vector3 *move = contactArray[maxIndex].particleMovement;
		for (i = 0; i < numContacts; i++)
		{
			if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0])
			{
				contactArray[i].penetration -= move[0] * contactArray[i].contactNormal;
			}
			else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1])
			{
				contactArray[i].penetration -= move[1] * contactArray[i].contactNormal;
			}
			if (contactArray[i].particle[1])
			{
				if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0])
				{
					contactArray[i].penetration += move[0] * contactArray[i].contactNormal;
				}
				else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1])
				{
					contactArray[i].penetration += move[1] * contactArray[i].contactNormal;
				}
			}
		}

		iterationsUsed++;
	}
}