#ifndef BODY_H
#define BODY_H

#include "../Math/core.h"

namespace Physics_Engine
{
	class RigidBody
	{
	protected:
		real inverseMass;
		Matrix3X3 inverseInertiaTensor;
		real linearDamping;
		real angularDamping;
		Vector3 position;
		Quaternion orientation;
		Vector3 velocity;
		Vector3 rotation;

		Matrix3X3 inverseInertiaTensorWorld;
		bool isAwake;
		bool canSleep;
		real motion;

		real sleepEpsolion;
		Matrix3X4 transformMatrix;
		Vector3 forceAccum;
		Vector3 torqueAccum;
		Vector3 acceleration;
		Vector3 lastFrameAcceleration;

	public:
		void calculateDerivedData();
		void integrate(real duration);
		void setMass(const real mass);
		real getMass() const;
		real getInverseMass() const;
		bool hasFiniteMass() const;
		Vector3 getLastFrameAcceleration() const;
		void getLastFrameAcceleration(Vector3 *lastFrameAcceleration) const;
		void setInertiaTensor(const Matrix3X3 &inertiaTensor);
		void getInverseIneritaTensor(Matrix3X3 *inverseInertiaTensor) const;
		Matrix3X3 getInverseIneritaTensor() const;
		void getInverseInertiaTensorWorld(Matrix3X3 *inverseInertiaTensor) const;
		Matrix3X3 getInverseInertiaTensorWorld() const;
		void setDamping(const real linearDamping, const real angularDamping);
		void setLinearDamping(const real linearDamping);
		real getLinearDamping() const;
		void setAngularDamping(const real angularDamping);
		real getAngularDamping() const;
		void clearAccumulators();
		void addForce(const Vector3 &force);
		void addForceAtPoint(const Vector3 &force, const Vector3 &point);
		void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
		void addTorque(const Vector3 &torque);
		void setVelocity(const real x, const real y, const real z);
		void setVelocity(const Vector3 &velocity);
		void getVelocity(Vector3 *velocity) const;
		Vector3 getVelocity() const;
		void addVelocity(const Vector3 &deltaVelocity);
		void setPosition(const Vector3 &position);
		void setPosition(const real x, const real y, const real z);
		void getPosition(Vector3 *position) const;
		Vector3 getPosition() const;
		void setAcceleration(const Vector3 &acceleration);
		void setAcceleration(const real x, const real y, const real z);
		void getAcceleration(Vector3 *acceleration) const;
		Vector3 getAcceleration() const;
		void setOrientation(const Quaternion &orientation);
		void setOrientation(const real r, const real i, const real j, const real k);
		void getOrientation(Quaternion *orientation) const;
		Quaternion getOrientation() const;
		void setRotation(const Vector3 &rotation);
		void setRotation(const real x, const real y, const real z);
		void getRotation(Vector3 *rotation);
		Vector3 getRotation() const;
		void addRotation(const Vector3 &deltaRotation);

		bool getAwake() const
		{
			return isAwake;
		}

		void setAwake(const bool isAwake = true);

		bool getCanSleep() const
		{
			return canSleep;
		}

		void setCanSleep(const bool canSleep = true);
		void getTransform(Matrix3X4 *transform) const;
		void getGLTransform(float matrix[16]) const;

		Matrix3X4 getTransform() const;
		Vector3 getPointInLocalSpace(const Vector3 &point) const;
		Vector3 getPointInWorldSpace(const Vector3 &point) const;
	};
}

#endif // BODY_H