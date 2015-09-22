#include "contacts.h"
#include <assert.h>

using namespace Physics_Engine;

void Contact::setBodyData(RigidBody *one, RigidBody *two, real friction, real restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

void Contact::calculateInternals(real duration)
{
	// Check if the first object is NULL, and swap if it is.
	if (!body[0])
		swapBodies();

	assert(body[0]);

	// Calculate a set of axis at the contact point.
	calculateContactBasis();

	// Store the relative position of the contact relative to each body.
	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if (body[1])
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();

	// Find the relative velocity of the bodies at the contact point.
	contactVelocity = calculateLocalVelocity(0, duration);
	if (body[1])
		contactVelocity -= calculateLocalVelocity(1, duration);

	// Calculate the desired change in velocity for resolution.
	calcualteDesiredDeltaVelocity(duration);
}

void Contact::swapBodies()
{
	contactNormal *= -1;

	RigidBody *temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

void Contact::matchAwakeState()
{
	// Collisions with the world never cause a body to wake up.
	if (!body[1])
		return;

	bool bodyAwakeOne = body[0]->getAwake();
	bool bodyAwakeTwo = body[1]->getAwake();

	// Wake up only the sleeping one.
	if (bodyAwakeOne ^ bodyAwakeTwo)
	{
		if (bodyAwakeOne)
			body[1]->setAwake();
		else
			body[0]->setAwake();
	}
}

void Contact::calcualteDesiredDeltaVelocity(real duration)
{
	const static real velocityLimit = (real)0.25f;

	// Calculate the acceleration induced velocity accumulated this frame
	real velocityFromAcc = 0;

	if (body[0]->getAwake())
	{
		velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
	}

	if (body[1] && body[1]->getAwake())
	{
		velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;
	}

	// Ifthe velocity is very slow, limit the restitution.
	real thisRestitution = restitution;
	if (real_abs(contactVelocity.x) < velocityLimit)
	{
		thisRestitution = (real)0.0f;
	}

	/*
		Combine the bounce velocity with the removed
		acceleration velocity.
	*/
	desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
}

// --- pointVelRel = angularVel * pointRel + linearVel ---
Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real duration)
{
	RigidBody *thisBody = body[bodyIndex];

	// Work out the velocity of the contact point.
	Vector3 velocity = thisBody->getRotation().vectorProduct(relativeContactPosition[bodyIndex]);
	velocity += thisBody->getVelocity();

	// Turn the velocity into contact coordinates.
	Vector3 contactVelocity = contactToWorld.transformTranspose(velocity);

	/*
		Calculate the amount of velocity that is due 
		to forces without reactions.
	*/
	Vector3 accVelocity = thisBody->getLastFrameAcceleration() * duration;

	// Calculate the velocity in contact coordinates.
	accVelocity = contactToWorld.transformTranspose(accVelocity);

	/*
		We ignore any component of acceleration in the contact normal
		direction, we are only intrested in planar acceleration.
	*/
	accVelocity.x = 0;

	/*
		Add the planar velocities - if there's enough friction they
		will be removed during velocity resolution.
	*/
	contactVelocity += accVelocity;

	return contactVelocity;
}

/*
	Constructs an arbitrary orthonormal basis for the contact. This is
	stored as a 3X3 matrix, where each vector is a column (in other words,
	the matrix transforms contact space into world space). The x direction
	is generated from the contact normal, and the y and z directions are
	set to that they are at right angles to it.
*/
inline void Contact::calculateContactBasis()
{
	Vector3 contactTanget[2];

	// Check whether the z-axis is nearer to the x or y axis.
	if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
	{
		// Scaling factor to ensure the results are normalized.
		const real s = (real)1.0f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

		// The new x axis is at right angles to the world y axis.
		contactTanget[0].x = contactNormal.z * s;
		contactTanget[0].y = 0;
		contactTanget[0].z = -contactNormal.x * s;

		// The new y axis is at right angles to the new x and z axis.
		contactTanget[1].x = contactNormal.y * contactTanget[0].x;
		contactTanget[1].y = contactNormal.z * contactTanget[0].x - contactNormal.x * contactTanget[0].z;
		contactTanget[1].z = -contactNormal.y * contactTanget[0].x;
	}
	else
	{
		// Scaling factor to ensure the results are normalized.
		const real s = (real)1.0f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

		// The new x axis is a right angles to the world x axis.
		contactTanget[0].x = 0;
		contactTanget[0].y = -contactNormal.z * s;
		contactTanget[0].z = contactNormal.y * s;

		// The new y axis is at right angles to the new x and z axis.
		contactTanget[1].x = contactNormal.y * contactTanget[0].z - contactNormal.z * contactTanget[0].y;
		contactTanget[1].y = -contactNormal.x * contactTanget[0].z;
		contactTanget[1].z = contactNormal.x * contactTanget[0].y;
	}

	// Make a matrix from the three vectors.
	contactToWorld.setColumns(contactNormal, contactTanget[0], contactTanget[1]);
}

inline Vector3 Contact::calculateFrictionlessImpulse(Matrix3X3 *inverseInertiaTensor)
{
	Vector3 impulseContact;

	/*
		Building a vector that shows the change in velocity in 
		world space for a unit impulse in the direction of the 
		contact normal.
	*/
	Vector3 deltaVelocityWorld = relativeContactPosition[0].vectorProduct(contactNormal);
	deltaVelocityWorld = inverseInertiaTensor[0].transform(deltaVelocityWorld);
	deltaVelocityWorld = deltaVelocityWorld.vectorProduct(relativeContactPosition[0]);

	// Work out the change in velocity in contact coordiantes.
	real deltaVelocity = deltaVelocityWorld * contactNormal;

	// Add the linear component of velocity change.
	deltaVelocity += body[0]->getInverseMass();

	// Check if we need to the second body's data.
	if (body[1])
	{
		// Go through the same transformation sequnce again.
		Vector3 deltaVelocityWorld = relativeContactPosition[1].vectorProduct(contactNormal);
		deltaVelocityWorld = inverseInertiaTensor[1].transform(deltaVelocityWorld);
		deltaVelocityWorld = deltaVelocityWorld.vectorProduct(relativeContactPosition[1]);

		// Work out the change in velocity in contact coordiantes.
		deltaVelocity += deltaVelocityWorld * contactNormal;

		// Add the linear component of velocity change.
		deltaVelocity += body[1]->getInverseMass();
	}

	/*
		The contact normal is along the x axis.
	*/

	// Calculate the required size of the impulse.
	impulseContact.x = desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0;
	impulseContact.z = 0;

	return impulseContact;
}

inline Vector3 Contact::calculateFrictionImpulse(Matrix3X3 * inverseInertiaTensor)
{
	Vector3 impulseContact;
	real inverseMass = body[0]->getInverseMass();

	/*
		The equivalent of a cross product in matrices is multiplication
		by a skew symmetric matrix. We build the matrix for converting 
		between linear and angular quantities.
	*/
	Matrix3X3 impulseToTorgue;
	impulseToTorgue.setSkewSymmetric(relativeContactPosition[0]);

	/*
		Build the matrix to convert contact impulse to change in velocity
		in world coordinates.
	*/
	Matrix3X3 deltaVelocityWorld = impulseToTorgue;
	deltaVelocityWorld *= inverseInertiaTensor[0];
	deltaVelocityWorld *= impulseToTorgue;
	deltaVelocityWorld *= -1;

	// Check if we need to add body's two's data.
	if (body[1])
	{
		// Set the cross product matrix.
		impulseToTorgue.setSkewSymmetric(relativeContactPosition[1]);

		// Calculate the velocity change matrix.
		Matrix3X3 deltaVelocityWorld_2 = impulseToTorgue;
		deltaVelocityWorld_2 *= inverseInertiaTensor[1];
		deltaVelocityWorld_2 *= impulseToTorgue;
		deltaVelocityWorld_2 *= -1;

		// Add to the total delta velocity
		deltaVelocityWorld += deltaVelocityWorld_2;

		// Add the inverse mass
		inverseMass += body[1]->getInverseMass();
	}

	// Do a change of basic to convert into contact coordinates.
	Matrix3X3 deltaVelocity = contactToWorld.transpose();
	deltaVelocity *= deltaVelocityWorld;
	deltaVelocity *= contactToWorld;

	// Add in the linear velocity change.
	deltaVelocity.data[0] += inverseMass;
	deltaVelocity.data[4] += inverseMass;
	deltaVelocity.data[8] += inverseMass;

	// Invert to get the impulse needed per unit velocity.
	Matrix3X3 impulseMatrix = deltaVelocity.inverse();

	// Find the target velocities to kill.
	Vector3 velocityKill(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);

	// Find the impulse to kill the target velocities.
	impulseContact = impulseMatrix.transform(velocityKill);

	// Check for exceeding friction.
	real planarImpulse = real_sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);

	if (planarImpulse > impulseContact.x * friction)
	{
		// We need to use dynamic friction.
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;

		impulseContact.x = deltaVelocity.data[0] +
			deltaVelocity.data[1] * friction * impulseContact.y +
			deltaVelocity.data[2] * friction * impulseContact.z;
		impulseContact.x = desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= friction * impulseContact.x;
		impulseContact.z *= friction * impulseContact.x;
	}
	return impulseContact;
}

void Contact::applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
	/*
		Get holds of the inverse mass and inverse inertia tensor
		both in world coordinates.
	*/
	Matrix3X3 inverseInertiaTensor[2];
	body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
	if (body[1])
		body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

	// We will calculate the impulse for each contact axis.
	Vector3 impulseContact;

	if (friction == (real)0.0)
	{
		// Use the short format for frictionless contacts
		impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		/*
			Otherwise we may have impulses that are not in the direction of 
			the contact, so we need the more complex version.
		*/
		impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
	}

	// Convert impulse to world coordinates.
	Vector3 impulse = contactToWorld.transform(impulseContact);
	
	// Split in the impulse into linear and rotational components.
	Vector3 impulsiveTorque = relativeContactPosition[0].vectorProduct(impulse);
	rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
	velocityChange[0].clear();
	velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());

	// Apply the changes
	body[0]->addVelocity(velocityChange[0]);
	body[0]->addRotation(rotationChange[0]);

	if (body[1])
	{
		Vector3 impulsiveTorque = impulse.vectorProduct(relativeContactPosition[1]);
		rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
		velocityChange[1].clear();
		velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());

		// Apply the changes
		body[1]->addVelocity(velocityChange[1]);
		body[1]->addRotation(rotationChange[1]);
	}
}

void Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration)
{
	const real angularLimit = (real)0.2f;
	real angularMove[2];
	real linearMove[2];

	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];

	/*
		We need to work out the inertia of each object in the direction
		of the contact normal, due to angular inertia only.
	*/
	for (unsigned i = 0; i < 2; i++)
	{
		if (body[i])
		{
			Matrix3X3 inverseInertiaTensor;
			body[i]->getInverseIneritaTensor(&inverseInertiaTensor);

			/*
				Use the same procedure as for calculating frictionless
				velocity change to work out the angular inertia.
			*/
			Vector3 angularInertiaWorld = relativeContactPosition[i].vectorProduct(contactNormal);
			angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
			angularInertiaWorld = angularInertiaWorld.vectorProduct(relativeContactPosition[i]);
			angularInertia[i] = angularInertiaWorld * contactNormal;

			// The linear component is simply the inverse mass.
			linearInertia[i] = body[i]->getInverseMass();

			// Keep track of the total inertia from all components.
			totalInertia += linearInertia[i] + angularInertia[i];
		}

		/*			---- Must be a better way to write this, slower code, maybe not cache friendly ----
			We break the loop here so that the totalInertia value is
			completely created (by both iterations) before continuing.
		*/
	}

	// Loop through again calculating and applying changes.
	for (unsigned i = 0; i < 2; i++)
	{
		if (body[i])
		{
			/*
				The linear and angular movements required are in proportion
				to the two inverse masses.
			*/
			real sign = (i == 0) ? 1 : -1;
			angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
			linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

			/*
				To avoid angular projections that are too great (when mass is large
				but inertia tensor is small) limit the angular move.
			*/
			Vector3 projection = relativeContactPosition[i];
			projection.addScaledVector(contactNormal, -relativeContactPosition[i].scalarProduct(contactNormal));
			
			/*
				We use small angle approximation for the sine of the angle
				(i.e the magnitude would be sine(angularLimit) * projection.magnitude
				but we approximate sine(angularLimit) to angularLimit).
			*/
			real maxMagnitude = angularLimit * projection.magnitude();

			if (angularMove[i] < -maxMagnitude)
			{
				real totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude)
			{
				real totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			/*
				We have the linear amount of movement required by turning 
				the rigid body (in angularMove[i]). We now need to calcualte
				the desired rotation to achieve that.
			*/
			if (angularMove[i] == 0)
			{
				// Easy case - no angular movement means no roation.
				angularChange[i].clear();
			}
			else
			{
				// Work out the direction we would like to rotate in.
				Vector3 targetAngualrDirection = relativeContactPosition[i].vectorProduct(contactNormal);
				
				Matrix3X3 inverseInertiaTensor;
				body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

				// Work out the direction we would need to rotate to achieve that.
				angularChange[i] = inverseInertiaTensor.transform(targetAngualrDirection) *
					(angularMove[i] / angularInertia[i]);
			}

			/*
				Velocity change is easier - it is just the linear movement
				along the contact normal.
			*/
			linearChange[i] = contactNormal * linearMove[i];

			/*
				Now we can start to apply the values we have calculated.
				Apply the linear movement.
			*/
			Vector3 pos;
			body[i]->getPosition(&pos);
			pos.addScaledVector(contactNormal, linearMove[i]);
			body[i]->setPosition(pos);

			// And the change in orientation.
			Quaternion q;
			body[i]->getOrientation(&q);
			q.addScaledVector(angularChange[i], ((real)1.0));
			body[i]->setOrientation(q);

			/*
				We need to calculate the derived data for any body that is
				asleep, so that the changes are reflected in the object's
				data. Otherwise the resolution will not change the position
				of the object, and the next collision detection round will
				have the same penetation.
			*/
			if (!body[i]->getAwake())
				body[i]->calculateDerivedData();
		}
	}
}


ContactResolver::ContactResolver(unsigned iterations, real velocityEpsilon, real positionEpsilon)
{
	setIterations(iterations, iterations);
	setEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon, real positionEpsilon)
{
	setIterations(velocityIterations, positionIterations);
	setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned velocityIterations, unsigned positionIterations)
{
	ContactResolver::velocityIterations = velocityIterations;
	ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setIterations(unsigned iterations)
{
	setIterations(iterations, iterations);
}

void ContactResolver::setEpsilon(real velocityEpsilion, real positionEpsilon)
{
	ContactResolver::velocityEpsilon = velocityEpsilion;
	ContactResolver::positionEpsilon = positionEpsilon;
}

/*
	********************************************************
					The contact resolver.
	********************************************************
*/
void ContactResolver::resolveContacts(Contact * contacts, unsigned numContacts, real duration)
{
	// Make sure we have something to do.
	if (numContacts == 0)
		return;

	if (!isValid())
		return;

	// Prepare the contacts for processing.
	prepareContacts(contacts, numContacts, duration);

	// Resolve the interpenetration problem with the contacts.
	adjustPositions(contacts, numContacts, duration);

	// Resolve the velocity problems with the contacts.
	adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts, unsigned numContacts, real duration)
{
	// Generate contact velocity and axis information.
	Contact* lastContact = contacts + numContacts;
	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		// Calculate the internal contact data (inertia, basis, etc.)
		contact->calculateInternals(duration);
	}
}

void ContactResolver::adjustVelocities(Contact *contacts, unsigned numContacts, real duration)
{
	Vector3 velocityChange[2], rotationChange[2];
	Vector3 deltaVelocity;

	// Iteratively handle impaces in order of severity.
	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		// Find contact with maximum magnitude of probable velocity change.
		real max = velocityEpsilon;
		unsigned index = numContacts;
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (contacts[i].desiredDeltaVelocity > max)
			{
				max = contacts[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts)
			break;

		// Match the awake state at the contact.
		contacts[index].matchAwakeState();

		contacts[index].applyVelocityChange(velocityChange, rotationChange);

		/*
			With the change in velocity of the two bodies, the update of
			contact velocities means that some of the relative closing
			velocities need recomputing.
		*/
		for (unsigned i = 0; i < numContacts; i++)
		{
			// Check each body in the contact.
			for (unsigned bodyIndex = 0; bodyIndex < 2; bodyIndex++)
				if(contacts[index].body[bodyIndex])
			{
				/*
					Check for a math with each body in the 
					newly resolved contact.
				*/
				for (unsigned d = 0; d < 2; d++)
				{
					if (contacts[i].body[bodyIndex] == contacts[index].body[d])
					{
						deltaVelocity = velocityChange[d] + rotationChange[d].
							vectorProduct(contacts[i].relativeContactPosition[bodyIndex]);

						/*
							The sign of the change is negative if we are dealing
							with the second body in a contact.
						*/
						contacts[i].contactVelocity += contacts[i].contactToWorld.
							transformTranspose(deltaVelocity) * (bodyIndex ? -1 : 1);

						contacts[i].calcualteDesiredDeltaVelocity(duration);
					}
				}
			}
		}
		velocityIterationsUsed++;
	}
}

void ContactResolver::adjustPositions(Contact *contacts, unsigned numContacts, real duration)
{
	unsigned i, index;
	Vector3 linearChange[2], angularChange[2];
	real max;
	Vector3 deltaPosition;

	// Iteratively resolve interpenetrations in order of severity.
	positionIterationsUsed = 0;
	while (positionIterationsUsed < positionIterations)
	{
		// Find the biggest penetration.
		max = positionEpsilon;
		index = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			if (contacts[i].penetration > max)
			{
				max = contacts[i].penetration;
				index = i;
			}
		}
		if (index == numContacts)
			break;

		// Match the awake state at the contact.
		contacts[index].matchAwakeState();

		// Resolve the penetration.
		contacts[index].applyPositionChange(linearChange, angularChange, max);

		/*
			Again this action may have changed th penetration of 
			other bodies, so we update contacts.
		*/
		for (i = 0; i < numContacts; i++)
		{
			// Check each body in the contact.
			for (unsigned bodyIndex = 0; bodyIndex < 2; bodyIndex++)
				if (contacts[i].body[bodyIndex])
			{
				// Check for a match with each body in the newly resolved contact.
				for (unsigned d = 0; d < 2; d++)
				{
					if (contacts[i].body[bodyIndex] == contacts[index].body[d])
					{
						deltaPosition = linearChange[d] + angularChange[d].vectorProduct(contacts[i].relativeContactPosition[bodyIndex]);

						/*
							The sign of the change is positve if we are dealing wtih 
							the second body in a contact, and negative otherwise
							(because we are subtracting and resolution).
						*/
						contacts[i].penetration += deltaPosition.scalarProduct(contacts[i].contactNormal) * (bodyIndex ? 1 : -1);
					}
				}
			}
		}
		positionIterationsUsed++;
	}
}