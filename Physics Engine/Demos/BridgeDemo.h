#ifndef BRIDGE_DEMO_H
#define BRIDGE_DEMO_H

#include "../Math/core.h"
#include "../Dynamics/plinks.h"
#include "../Dynamics/pworld.h"

using namespace Physics_Engine;

class BridgeDemo
{
	ParticleCableConstraint *supports;
	ParticleCable *cables;
	ParticleRod *rods;

	Vector3 massPos;
	Vector3 massDisplayPos;

	ParticleWorld world;
	Particle *particleArray;

public:
	BridgeDemo();
	~BridgeDemo();

	void Render();
	void ResetDemo();
	void Update(float duration);

	void DrawGrid(Physics_Engine::Vector3 &position);
};

#endif