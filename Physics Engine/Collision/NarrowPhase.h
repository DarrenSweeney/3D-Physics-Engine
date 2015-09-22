#ifndef NARROWPHASE_H
#define NARROWPHASE_H

#include "contacts.h"
#include "../Dynamics/body.h"

namespace Physics_Engine
{
	class CollisionPrimitive
	{
	public:
		/*
			This class exists to help the collision dector 
			and intersetion routines, so they should have
			access to its data.

			A friend class in C++ can access the "private" and "protected" 
			members of the class in which it is declared as a friend.
		*/
		friend class IntersectionTests;
		friend class CollisionDectector;

		RigidBody *body;

		/* 
			The offset of this primitive from the given rigid body.
			Offset is the rotation and translation.
		*/
		Matrix3X4 offset;

		// Calculates the internals for the primitive.
		void calculateInternals();

		/*
			This is a convenience function to allow access to the 
			axis vectors in the transform for this primitive.
		*/
		Vector3 getAxis(unsigned index) const
		{
			return  transform.getAxisVector(index);
		}

		/*
			Returns the resultant transform of the primitive, calculated from
			the combined offset of the primitive and the transform
			(orientation + position) of the rigid body to which
			it is attached.
		*/
		const Matrix3X4 &getTransform() const
		{
			return transform;
		}

	protected:
		/*
			The resultant transform of the primitive. This is
			calculated by combining the offset of the primitive
			with the transform of the rigid body.
		*/
		Matrix3X4 transform;
	};

	class CollisionSphere : public CollisionPrimitive
	{
	public:
		real radius;
	};

	class CollisionPlane : public CollisionPrimitive
	{
	public:
		Vector3 normal;

		// The distance of the plane from the origin.
		real offset;
	};

	class CollisionBox : public CollisionPrimitive
	{
	public:
		Vector3 halfSize;
	};

	/*
		A wrapper class that holds fast intersection tests. These
		can be used to drive the broad phase collision dectection 
		system or as an early out in the full collision tests below.
	*/
	class IntersectionTests
	{
	public:
		static bool sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane);

		static bool sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two);

		static bool boxAndBox(const CollisionBox &one, const CollisionBox &two);

		static bool boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);
	};

	/*
		A helper structure that contains information for the detector to use
		in the building its contact data.
	*/
	struct CollisionData
	{
		/*
			Holds the base of the collision data: the first contact
			in the array. This is used so that the contact pointer (below)
			can be incremented each time a contact is detected, while
			this pointer points the the first contact found.
		*/
		Contact *contactArray;

		// Holds the data contact array to write into.
		Contact *contacts;

		// Holds the maximum number of contacts the array can take.
		int contactsLeft;

		unsigned contactCount;
		real friction;
		real restitution;

		/*
			Holds the collision telerance, even uncolliding objects this
			close should have collisions generated.
		*/
		real tolerance;

		// Checks if there are more contacts avaible in the contact data.
		bool hasMoreContacts()
		{
			return contactsLeft > 0;
		}

		// Resets the data so that it has no used contacts recorded.
		void reset(unsigned maxContacts)
		{
			contactsLeft = maxContacts;
			contactCount = 0;
			contacts = contactArray;
		}

		/*
			Notifies the data that the given number of contacts have
			been added.
		*/
		void addContacts(unsigned count)
		{
			// Reduce the number of contacts remaining, add number used.
			contactsLeft -= count;
			contactCount += count;

			// Move the array forward
			contacts += count;
		}
	};

	/*
		Each of the functions has the same format: it takes the details
		of two objects, and a pointer to a contact array to fill. It
		returns the number of contacts it wrote into the array.
	*/
	class CollisionDectector
	{
	public:
		static unsigned sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two, CollisionData *data);

		/*
			The half space is treated as if the whole region on the back plane is solid. 
			An object interpenetrating will always have it's contact normal pointing in
			the direction of the plane normal. Even if the object is completely behind
			the plane, the contact will be generated.
		*/
		static unsigned sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data);

		static unsigned boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane, CollisionData *data);

		static unsigned boxAndSphere(const CollisionBox &box, const CollisionSphere &sphere, CollisionData *data);

		static unsigned boxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data);
	};
}
#endif