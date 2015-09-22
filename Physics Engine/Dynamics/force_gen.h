#ifndef FORCE_GEN_H
#define FORCE_GEN_H

#include "../Dynamics/Body.h"
#include <vector>

namespace Physics_Engine
{
	class ForceGenerator
	{
	public:
		/*
			Overload this in implementations of the interface to calculate
			and update the force applied to given rigid body.
		*/
		virtual void updateForce(RigidBody *body, real duration) = 0;
	};

	class Gravity : public ForceGenerator
	{
		Vector3 gravity;

	public:
		Gravity(const Vector3 &gravity);

		virtual void updateForce(RigidBody *body, real duration);
	};

	class Spring : public ForceGenerator
	{
		Vector3 connectionPoint;
		Vector3 otherConnectionPoint;
		RigidBody *other;
		real springConstant;
		real restLength;

	public:
		Spring(const Vector3 &localConnectionPt, RigidBody *other, const Vector3 &otherConnectionPt, real springConstant, real restLength);

		virtual void updateForce(RigidBody *body, real duration);
	};

	/*
		A force generator that applies an aeordynamic force.
	*/
	class Aero : public ForceGenerator
	{
	protected:
		Matrix3X3 tensor;
		Vector3 position;
		const Vector3* windspeed;

	public:
		Aero(const Matrix3X3 &tensor, const Vector3 &position,
			const Vector3 *windspeed);

		virtual void updateForce(RigidBody *body, real duration);

	protected:
		void updateForceFromTensor(RigidBody *body, real duration, const Matrix3X3 &tensor);
	};

	/*
		A force generator with a control aerodynamic surface. This
		requires three inertia tensors, for the two extremes and
		'resting' position of the control surface. The latter tensor
		is one inherited from the base class, while the two extremes
		are defined in this class.
	*/
	class AeroControl : public Aero
	{
	protected:
		Matrix3X3 maxTensor;
		Matrix3X3 minTensor;
		real controlSetting;

	private:
		Matrix3X3 getTensor();

	public:
		AeroControl(const Matrix3X3 &base, const Matrix3X3 &min, const Matrix3X3 &max,
			const Vector3 &position, const Vector3 *windspeed);
		void setControl(real value);

		virtual void updateForce(RigidBody *body, real duration);
	};

	/*
		A force generator to apply a buoyant force to a rigid body.
	*/
	class Buoyancy : public ForceGenerator
	{
		real maxDepth;
		real volume;
		real waterHeight;
		real liquidDensity;
		Vector3 centerOfBuoyancy;

	public:
		Buoyancy(const Vector3 &centerBuoyancy, real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);

		virtual void updateForce(RigidBody *body, real duration);
	};

	/*
		A force generator with an aerodynamic surface that can be
		reoriented relative to it's rigid body.
	*/
	class AngledAero : public Aero
	{
		Quaternion orientation;

	public:
		AngledAero(const Matrix3X3 &tensor, const Vector3 &position, const Vector3 *windspeed);
		void setOrientation(const Quaternion &quat);

		virtual void updateForce(RigidBody *body, real duration);
	};

	/*
		Holds all the force generators and the bodies they apply to.
	*/
	class ForceRegistry
	{
	protected:
		struct ForceRegistation
		{
			RigidBody *body;
			ForceGenerator *forceGen;
		};

		typedef std::vector<ForceRegistation> ForceRegistations;
		ForceRegistations registations;

	public:
		void add(RigidBody *body, ForceGenerator *forceGen);
		void remove(RigidBody *body, ForceGenerator *forceGen);
		void clear();
		void updateForces(real duration);
	};
}
#endif