#include "force_gen.h"

using namespace Physics_Engine;

void Gravity::updateForce(RigidBody *body, real duration)
{
	// Check that we do not have infinite mass.
	if (!body->hasFiniteMass())
		return;

	// Apply the mass-scaled force to the body.
	body->addForce(gravity * body->getMass());
}

void Spring::updateForce(RigidBody *body, real duration)
{
	// Calculate the two ends in world space
	Vector3 lws = body->getPointInWorldSpace(connectionPoint);
	Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);

	// Calculate the final force and apply it.
	Vector3 force = lws - ows;

	// Calculate the magnitude of the force.
	real magnitde = force.magnitude();
	magnitde = real_abs(magnitde - restLength);
	magnitde *= springConstant;

	// Calculate the final force and apply it.
	force.normalise();
	force *= -magnitde;
	body->addForceAtPoint(force, lws);
}

void Aero::updateForce(RigidBody *body, real duration)
{
	Aero::updateForceFromTensor(body, duration, tensor);
}

Aero::Aero(const Matrix3X3 &tensor, const Vector3 &position,
	const Vector3 *windspeed)
{
	Aero::tensor = tensor;
	Aero::position = position;
	Aero::windspeed = windspeed;
}

void Aero::updateForceFromTensor(RigidBody *body, real duration, const Matrix3X3 &tensor)
{
	//   Calculate total velocity (wind speed and body's velocity)
	Vector3 velocity = body->getVelocity();
	velocity += *windspeed;

	// Calculate the velocity in body coordinates
	Vector3 bodyVel = body->getTransform().transformInverseDirection(velocity);

	// Calculate the force in body coordinates
	Vector3 bodyForce = tensor.transform(bodyVel);
	Vector3 force = body->getTransform().transformDirection(bodyForce);

	body->addForceAtBodyPoint(force, position);
}

AeroControl::AeroControl(const Matrix3X3 &base, const Matrix3X3 &min, const Matrix3X3 &max,
	const Vector3 &position, const Vector3 *windspeed)
	:
	Aero(base, position, windspeed)
{
	AeroControl::minTensor = min;
	AeroControl::maxTensor = max;
	controlSetting = 0.0f;
}

void AeroControl::setControl(real value)
{
	controlSetting = value;
}

Matrix3X3 AeroControl::getTensor()
{
	if (controlSetting <= -1.0f)
		return minTensor;
	else if (controlSetting >= 1.0f)
		return maxTensor;
	else if (controlSetting < 0)
	{
		//														Don't want to divide by negative ratio
		return Matrix3X3::linearInterpolate(minTensor, tensor, controlSetting + 1.0f);
	}
	else if (controlSetting > 0)
	{
		return Matrix3X3::linearInterpolate(tensor, maxTensor, controlSetting);
	}
	else 
		return tensor;
}

void AeroControl::updateForce(RigidBody *body, real duration)
{
	Matrix3X3 tensor = getTensor();
	updateForceFromTensor(body, duration, tensor);
}

Buoyancy::Buoyancy(const Vector3 &centerBuoyancy, real maxDepth, real volume,
					real waterHeight, real liquidDensity /*= 1000.0f*/)
{
	centerOfBuoyancy = centerBuoyancy;
	Buoyancy::maxDepth = maxDepth;
	Buoyancy::volume = volume;
	Buoyancy::waterHeight = waterHeight;
	Buoyancy::liquidDensity = liquidDensity;
}

void Buoyancy::updateForce(RigidBody *body, real duration)
{
	// Calculate the submersion depth
	Vector3 pointInWorld = body->getPointInWorldSpace(centerOfBuoyancy);
	real depth = pointInWorld.y;

	// Check if we are out of the water
	if (depth >= waterHeight + maxDepth)
		return;

	Vector3 force(0, 0, 0);

	// Check if we are at maximum depth
	if (depth <= waterHeight - maxDepth)
	{
		force.y = liquidDensity * volume;
		body->addForceAtBodyPoint(force, centerOfBuoyancy);

		return;
	}

	// Otherwise we are partly submerged
	force.y = liquidDensity * volume *
		(depth - maxDepth - waterHeight) / 2 * maxDepth;
	body->addForceAtBodyPoint(force, centerOfBuoyancy);
}

AngledAero::AngledAero(const Matrix3X3 &tensor, const Vector3 &position, const Vector3 *windspeed)
: Aero(tensor, position, windspeed)
{
	AngledAero::tensor = tensor;
	AngledAero::position = position;
	AngledAero::windspeed = windspeed;
}

void AngledAero::updateForce(RigidBody *body, real duration)
{
	updateForceFromTensor(body, duration, tensor);
}

void ForceRegistry::updateForces(real duration)
{
	ForceRegistations::iterator i = registations.begin();
	for (; i != registations.end(); i++)
	{
		i->forceGen->updateForce(i->body, duration);
	}
}

void ForceRegistry::add(RigidBody *body, ForceGenerator *forceGen)
{
	ForceRegistry::ForceRegistation registration;
	registration.body = body;
	registration.forceGen = forceGen;
	registations.push_back(registration);
}

void ForceRegistry::remove(RigidBody *body, ForceGenerator *forceGen)
{

}

void ForceRegistry::clear()
{

}