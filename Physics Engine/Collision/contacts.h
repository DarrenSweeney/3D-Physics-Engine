#ifndef CONTACT_H
#define CONTACT_H

#include "../Dynamics/body.h"
#include "../Math/core.h"

namespace Physics_Engine
{
	/*
		A contact represents two bodies in contact, Resolving a
		contact removes their interpenetration, and applies sufficient
		impulse to keep them apart. Colliding bodies may also rebound.
		Contacts can be used to represent a positional joints, by making
		the contact constraint keep the bodies in their correct orientation.
	*/
	class Contact
	{
		/*
			The contact resolver object needs access into the contacts
			to set and effect the contact.
		*/
		friend class ContactResolver;

	public:
		/*
			Holds the bodies that are involved in the contact. The 
			second of these can be NULL, for contacts with the scenery.
		*/
		RigidBody *body[2];
		real friction;
		real restitution;
		Vector3 contactPoint;
		Vector3 contactNormal;
		real penetration;

		void setBodyData(RigidBody *one, RigidBody *two, real friction, real restitution);


	protected:
		Matrix3X3 contactToWorld;
		Vector3 contactVelocity;
		real desiredDeltaVelocity;
		Vector3 relativeContactPosition[2];

	protected:
		void calculateInternals(real duration);
		void swapBodies();
		void matchAwakeState();
		void calcualteDesiredDeltaVelocity(real duration);
		Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);
		void calculateContactBasis();
		void applyImpulse(const Vector3 &impluse, RigidBody *body,
			Vector3 *velocityChange, Vector3 *rotationChange);

		void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
		void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);
		Vector3 calculateFrictionlessImpulse(Matrix3X3 *inverseInertiaTensor);
		Vector3 calculateFrictionImpulse(Matrix3X3 *inverseInertiaTensor);
	};

	/*
		The contact resolution routine. One resolver instance can 
		be shared for the entire simulation, as long as you need
		roughly the same parameters each time (which is normal).
	*/
	class ContactResolver
	{
	protected:
		unsigned velocityIterations;
		unsigned positionIterations;
		real velocityEpsilon, positionEpsilon;

	public:
		unsigned velocityIterationsUsed;
		unsigned positionIterationsUsed;

	private:
		// Keeps track of whether the internal setting are valid.
		bool validSettings;

	public:
		ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01,
			real positionEpsilon = (real)0.01);

		ContactResolver(unsigned velocityIterations, unsigned positionIterations,
			real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);

		bool isValid()
		{
			return (velocityIterations > 0) && (positionIterations > 0) &&
				(positionEpsilon >= 0.0f) && (positionEpsilon >= 0.0f);
		}

		void setIterations(unsigned velocityIterations, unsigned positionIterations);

		void setIterations(unsigned iterations);
		void setEpsilon(real velocityEpsilion, real positionEpsilon);

		/*
			Resolves a set of contacts for both penetration and velocity.
			Contacts that cannot interact with each other should be
			passed to seperate calls to resolveContacts, as the resolution
			algorithm takes much longer for lots of contacts than it does
			for the same number of contacts in small sets.
		*/
		void resolveContacts(Contact *contactArray, unsigned numContacts, real duration);

	protected:
		void prepareContacts(Contact *contactArray, unsigned numContacts,
			real duration);
		void adjustVelocities(Contact *contacts, unsigned numContacts,
			real duration);
		void adjustPositions(Contact *contacts, unsigned numContacts,
			real duration);
	};


	/*
		This is the basic polymorphic interface for contact generators
		applying to rigid bodies.
	*/
	class ContactGenerator
	{
	public:
		/*
			Fills the given contact structure with the generated contact.
			The contact pointer should point to the first available contact
			in a contact array, where limit is the maximum number of contacts
			in the array that can be written to. The method returns he number
			of contacts that have been written.
		*/
		virtual unsigned addContact(Contact *contact, unsigned limit) const  = 0;
	};
}
#endif