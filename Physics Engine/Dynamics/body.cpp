#include "body.h"
#include <assert.h>
#include <iostream>

using namespace Physics_Engine;

void RigidBody::integrate(real duration)
{
	if (!isAwake)
		return;

	// Calculate linear acceleration from force inputs.
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

	// Calculate the angular acceleration from torque inputs.
	Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

	// Ajust velocities.
	// Update the linear velocity from both acceleration and impulse.
	velocity.addScaledVector(lastFrameAcceleration, duration);

	// Update the angular velocity from both acceleration and impluse.sa
	rotation.addScaledVector(angularAcceleration, duration);

	// Impose drag.
	velocity *= real_pow(linearDamping, duration);
	rotation *= real_pow(angularDamping, duration);

	// Adjust positions
	// Update linear position
	position.addScaledVector(velocity, duration);

	// Update angular position
	orientation.addScaledVector(rotation, duration);

	// Normalize the orientation and update the matrices with the
	// new position and orientation.
	calculateDerivedData();

	if (canSleep)
	{
		//	E_k = 1/2 mv^2
		real kineticEnrgy = inverseMass * (velocity * velocity);

		if (kineticEnrgy <= 0.00001)
		{
			setAwake(false);
		}
	}

	clearAccumulators();
}

void RigidBody::setAwake(const bool awake)
{
	if (awake)
	{
		isAwake = true;
		motion = sleepEpsolion * 2.0f;
	}
	else
	{
		isAwake = false;
		velocity.clear();
		rotation.clear();
	}
}

void RigidBody::setCanSleep(const bool canSleep)
{
	RigidBody::canSleep = canSleep;

	if (!canSleep && !isAwake)
		setAwake();
}

void RigidBody::setMass(const real mass)
{
	assert(mass != 0);
	inverseMass = ((real)1.0) / mass;
}

bool RigidBody::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

Vector3 RigidBody::getLastFrameAcceleration() const
{
	return lastFrameAcceleration;
}

void RigidBody::getLastFrameAcceleration(Vector3 *lastFrameAcceleration) const
{
	*lastFrameAcceleration = RigidBody::lastFrameAcceleration;
}

real RigidBody::getMass() const
{
	if (inverseMass == 0)
		return REAL_MAX;
	else
		return ((real)1.0) / inverseMass;
}

real RigidBody::getInverseMass() const
{
	return inverseMass;
}

void RigidBody::setInertiaTensor(const Matrix3X3 &inertiaTensor)
{
	inverseInertiaTensor.setInverse(inertiaTensor);
}

void RigidBody::getInverseIneritaTensor(Matrix3X3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

Matrix3X3 RigidBody::getInverseIneritaTensor() const
{
	return inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensorWorld(Matrix3X3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = RigidBody::inverseInertiaTensorWorld;
}

Matrix3X3 RigidBody::getInverseInertiaTensorWorld() const
{
	return inverseInertiaTensorWorld;
}

void RigidBody::addForce(const Vector3 &force)
{
	forceAccum += force;
	isAwake = true;
}

void RigidBody::getTransform(Matrix3X4 *transform) const
{
	*transform = RigidBody::transformMatrix;
}

void RigidBody::getGLTransform(float matrix[16]) const
{
	matrix[0] = (float)transformMatrix.data[0];
	matrix[1] = (float)transformMatrix.data[4];
	matrix[2] = (float)transformMatrix.data[8];
	matrix[3] = 0;

	matrix[4] = (float)transformMatrix.data[1];
	matrix[5] = (float)transformMatrix.data[5];
	matrix[6] = (float)transformMatrix.data[9];
	matrix[7] = 0;

	matrix[8] = (float)transformMatrix.data[2];
	matrix[9] = (float)transformMatrix.data[6];
	matrix[10] = (float)transformMatrix.data[10];
	matrix[11] = 0;

	matrix[12] = (float)transformMatrix.data[3];
	matrix[13] = (float)transformMatrix.data[7];
	matrix[14] = (float)transformMatrix.data[11];
	matrix[15] = 1;
}

Matrix3X4 RigidBody::getTransform() const
{
	return transformMatrix;
}

Vector3 RigidBody::getPointInLocalSpace(const Vector3 &direction) const
{
	return transformMatrix.transformInverse(direction);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &direction) const
{
	return transformMatrix.transform(direction);
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += pt % force;

	isAwake = true;
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 pt = getPointInWorldSpace(point);

	addForceAtPoint(force, pt);
}

void RigidBody::addTorque(const Vector3 &torque)
{
	torqueAccum += torque;
	isAwake = true;
}

void RigidBody::setDamping(const real linearDamping, const real angularDamping)
{
	RigidBody::linearDamping = linearDamping;
	RigidBody::angularDamping = angularDamping;
}

void RigidBody::setLinearDamping(const real linearDamping)
{
	RigidBody::linearDamping = linearDamping;
}

real RigidBody::getLinearDamping() const
{
	return linearDamping;
}

void RigidBody::setAngularDamping(const real angularDamping)
{
	RigidBody::angularDamping = angularDamping;
}

real RigidBody::getAngularDamping() const
{
	return angularDamping;
}

void RigidBody::setPosition(const Vector3& position)
{
	RigidBody::position = position;
}

void RigidBody::setPosition(const real x, const real y, const real z)
{
	position.x = x;
	position.y = y;
	position.z = z;
}

void RigidBody::getPosition(Vector3 *position) const
{
	*position = RigidBody::position;
}

Vector3 RigidBody::getPosition() const
{
	return position;
}

void RigidBody::setOrientation(const Quaternion &orientation)
{
	RigidBody::orientation = orientation;
	RigidBody::orientation.normalize();
}

void RigidBody::setOrientation(const real r, const real i, const real j, const real k)
{
	orientation.r = r;
	orientation.i = i;
	orientation.j = j;
	orientation.k = k;

	orientation.normalize();
}

void RigidBody::getOrientation(Quaternion *orientation) const
{
	*orientation = RigidBody::orientation;
}

Quaternion RigidBody::getOrientation() const
{
	return orientation;
}

void RigidBody::setVelocity(const real x, const real y, const real z)
{
	velocity.x = x;
	velocity.y = y;
	velocity.z = z;
}

void RigidBody::setVelocity(const Vector3 &velocity)
{
	RigidBody::velocity = velocity;
}

Vector3 RigidBody::getVelocity() const
{
	return velocity;
}

void RigidBody::getVelocity(Vector3 *velocity) const
{
	*velocity = RigidBody::velocity;
}

void RigidBody::addVelocity(const Vector3 &deltaVelocity)
{
	velocity += deltaVelocity;
}

void RigidBody::setAcceleration(const Vector3 &acceleration)
{
	RigidBody::acceleration = acceleration;
}

void RigidBody::setAcceleration(const real x, const real y, const real z)
{
	acceleration.x = x;
	acceleration.y = y;
	acceleration.z = z;
}

void RigidBody::getAcceleration(Vector3 *accleration) const
{
	*accleration = RigidBody::acceleration;
}

Vector3 RigidBody::getAcceleration() const
{
	return acceleration;
}

void RigidBody::setRotation(const Vector3 &rotation)
{
	RigidBody::rotation = rotation;
}

void RigidBody::setRotation(const real x, const real y, const real z)
{
	rotation.x = x;
	rotation.y = y;
	rotation.z = z;
}

void RigidBody::getRotation(Vector3 *rotation)
{
	*rotation = RigidBody::rotation;
}

Vector3 RigidBody::getRotation() const
{
	return rotation;
}

void RigidBody::addRotation(const Vector3 &deltaRotation)
{
	rotation += deltaRotation;
}

void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum.clear();
}

static inline void _calculateTransformMatrix(Matrix3X4 &transformMatrix, const Vector3 &position, const Quaternion &orientation)
{
	transformMatrix.data[0] = 1 - 2 * orientation.j*orientation.j -
		2 * orientation.k*orientation.k;
	transformMatrix.data[1] = 2 * orientation.i*orientation.j -
		2 * orientation.r*orientation.k;
	transformMatrix.data[2] = 2 * orientation.i*orientation.k +
		2 * orientation.r*orientation.j;
	transformMatrix.data[3] = position.x;

	transformMatrix.data[4] = 2 * orientation.i*orientation.j +
		2 * orientation.r*orientation.k;
	transformMatrix.data[5] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.k*orientation.k;
	transformMatrix.data[6] = 2 * orientation.j*orientation.k -
		2 * orientation.r*orientation.i;
	transformMatrix.data[7] = position.y;

	transformMatrix.data[8] = 2 * orientation.i*orientation.k -
		2 * orientation.r*orientation.j;
	transformMatrix.data[9] = 2 * orientation.j*orientation.k +
		2 * orientation.r*orientation.i;
	transformMatrix.data[10] = 1 - 2 * orientation.i*orientation.i -
		2 * orientation.j*orientation.j;
	transformMatrix.data[11] = position.z;
}

static inline void _transformInertiaTensor(Matrix3X3 &iitWorld, const Quaternion &q,
	const Matrix3X3 &iitBody, const Matrix3X4 &rotmat)
{
	real t4 = rotmat.data[0] * iitBody.data[0] +
		rotmat.data[1] * iitBody.data[3] +
		rotmat.data[2] * iitBody.data[6];
	real t9 = rotmat.data[0] * iitBody.data[1] +
		rotmat.data[1] * iitBody.data[4] +
		rotmat.data[2] * iitBody.data[7];
	real t14 = rotmat.data[0] * iitBody.data[2] +
		rotmat.data[1] * iitBody.data[5] +
		rotmat.data[2] * iitBody.data[8];

	real t28 = rotmat.data[4] * iitBody.data[0] +
		rotmat.data[5] * iitBody.data[3] +
		rotmat.data[6] * iitBody.data[6];
	real t33 = rotmat.data[4] * iitBody.data[1] +
		rotmat.data[5] * iitBody.data[4] +
		rotmat.data[6] * iitBody.data[7];
	real t38 = rotmat.data[4] * iitBody.data[2] +
		rotmat.data[5] * iitBody.data[5] +
		rotmat.data[6] * iitBody.data[8];

	real t52 = rotmat.data[8] * iitBody.data[0] +
		rotmat.data[9] * iitBody.data[3] +
		rotmat.data[10] * iitBody.data[6];
	real t57 = rotmat.data[8] * iitBody.data[1] +
		rotmat.data[9] * iitBody.data[4] +
		rotmat.data[10] * iitBody.data[7];
	real t62 = rotmat.data[8] * iitBody.data[2] +
		rotmat.data[9] * iitBody.data[5] +
		rotmat.data[10] * iitBody.data[8];

	iitWorld.data[0] = t4*rotmat.data[0] +
		t9*rotmat.data[1] +
		t14*rotmat.data[2];
	iitWorld.data[1] = t4*rotmat.data[4] +
		t9*rotmat.data[5] +
		t14*rotmat.data[6];
	iitWorld.data[2] = t4*rotmat.data[8] +
		t9*rotmat.data[9] +
		t14*rotmat.data[10];

	iitWorld.data[3] = t28*rotmat.data[0] +
		t33*rotmat.data[1] +
		t38*rotmat.data[2];
	iitWorld.data[4] = t28*rotmat.data[4] +
		t33*rotmat.data[5] +
		t38*rotmat.data[6];
	iitWorld.data[5] = t28*rotmat.data[8] +
		t33*rotmat.data[9] +
		t38*rotmat.data[10];

	iitWorld.data[6] = t52*rotmat.data[0] +
		t57*rotmat.data[1] +
		t62*rotmat.data[2];
	iitWorld.data[7] = t52*rotmat.data[4] +
		t57*rotmat.data[5] +
		t62*rotmat.data[6];
	iitWorld.data[8] = t52*rotmat.data[8] +
		t57*rotmat.data[9] +
		t62*rotmat.data[10];
}

void RigidBody::calculateDerivedData()
{
	orientation.normalize();

	// Calculate the transform matrix for the body.
	_calculateTransformMatrix(transformMatrix, position, orientation);

	// Calculate the inerita tensor in world space.
	_transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
}