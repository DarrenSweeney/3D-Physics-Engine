#include "BridgeDemo.h"
#include <GL\glut.h>
#include "../Math/precision.h"

#define ROD_COUNT 6
#define CABLE_COUNT 10
#define SUPPORT_COUNT 12

#define BASE_MASS 1 // 1kg
#define EXTRA_MASS 10 // 10kg

BridgeDemo::BridgeDemo() :
world(12 * 10),
cables(0), supports(0), rods(0), massPos(0, 0, 0.5f)
{
	particleArray = new Particle[12];
	for (unsigned int i = 0; i < 12; i++)
	{
		world.getParticles().push_back(particleArray + i);
	}

	for (unsigned i = 0; i < 12; i++)
	{
		unsigned x = (i % 12) / 2;
		particleArray[i].setPosition(
			Physics_Engine::real(i / 2)*2.0f - 5.0f,
			4,
			Physics_Engine::real(i % 2)*2.0f - 1.0f
			);
		particleArray[i].setVelocity(0, 0, 0);
		particleArray[i].setDamping(0.9f);
		particleArray[i].setAcceleration(Physics_Engine::Vector3(0, -9.81, 0));
		particleArray[i].clearAccumulator();
	}

	cables = new Physics_Engine::ParticleCable[CABLE_COUNT];
	for (unsigned i = 0; i < 10; i++)
	{
		cables[i].particle[0] = &particleArray[i];
		cables[i].particle[1] = &particleArray[i + 2];
		cables[i].maxLength = 1.9f;
		cables[i].restitution = 0.3f;
		world.getContactGenerators().push_back(&cables[i]);
		cables[i].particle[1]->setVelocity(Physics_Engine::Vector3(1, 2, 3));
	}

	supports = new Physics_Engine::ParticleCableConstraint[SUPPORT_COUNT];
	for (unsigned i = 0; i < SUPPORT_COUNT; i++)
	{
		supports[i].particle = particleArray + i;
		supports[i].anchor = Physics_Engine::Vector3(
			Physics_Engine::real(i / 2)*2.2f - 5.5f,
			6,
			Physics_Engine::real(i % 2)*1.6f - 0.8f
			);
		if (i < 6) supports[i].maxLength = Physics_Engine::real(i / 2)*0.5f + 3.0f;
		else supports[i].maxLength = 5.5f - Physics_Engine::real(i / 2)*0.5f;
		supports[i].restitution = 0.5f;
		world.getContactGenerators().push_back(&supports[i]);
	}

	rods = new Physics_Engine::ParticleRod[ROD_COUNT];
	for (unsigned i = 0; i < 6; i++)
	{
		rods[i].particle[0] = &particleArray[i * 2];
		rods[i].particle[1] = &particleArray[i * 2 + 1];
		rods[i].length = 2;
		world.getContactGenerators().push_back(&rods[i]);
	}

	for (unsigned i = 0; i < 12; i++)
	{
		particleArray[i].setMass(BASE_MASS);
	}
}

BridgeDemo::~BridgeDemo()
{
	if (cables)
		delete[] cables;
	if (rods)
		delete[] rods;
	if (supports)
		delete[] supports;

	delete[] particleArray;
}

void BridgeDemo::ResetDemo()
{
	for (unsigned i = 0; i < 12; i++)
	{
		unsigned x = (i % 12) / 2;
		particleArray[i].setPosition(
			Physics_Engine::real(i / 2)*2.0f - 5.0f,
			4,
			Physics_Engine::real(i % 2)*2.0f - 1.0f
			);
		particleArray[i].setVelocity(0, 0, 0);
		particleArray[i].setDamping(0.9f);
		particleArray[i].setAcceleration(Physics_Engine::Vector3(0, -9.81, 0));
		particleArray[i].clearAccumulator();
	}

	for (unsigned i = 0; i < 10; i++)
	{
		cables[i].particle[1]->setVelocity(Physics_Engine::Vector3(1, 2, 3));
	}
}

void BridgeDemo::DrawGrid(Physics_Engine::Vector3 &pos)
{
	// Drawing a Grid Plane
	glBegin(GL_LINES);
	unsigned width = 40;
	for (unsigned i = 0; i <= width; i += 2)
	{
		glColor3f(1, 0, 0);
		if (i == width / 2) { glColor3f(0, 0, .8); }
		else { glColor3f(0.8, 0.8, 0.8); }
		glVertex3f(i - pos.x, pos.y, 0 - pos.z);
		glVertex3f(i - pos.x, pos.y, width - pos.z);
		if (i == width / 2) { glColor3f(.8, 0, 0); }
		else { glColor3f(0.8, 0.8, 0.8); }
		glVertex3f(-pos.x, pos.y, i - pos.z);
		glVertex3f(width - pos.x, pos.y, i - pos.z);
	};
	glEnd();
}

void BridgeDemo::Render()
{
	ParticleWorld::Particles &particles = world.getParticles();

	for (ParticleWorld::Particles::iterator p = particles.begin();
		p != particles.end(); p++)
	{
		const Vector3 &pos = (*p)->getPosition();

		glColor3f(0, 0, 0);
		glPushMatrix();
		glTranslatef(pos.x, pos.y, pos.z);
		glutSolidSphere(0.1f, 20, 20);
		glPopMatrix();

		// Draw shadows of particles
		glColor3f(0.75f, 0.75f, 0.75f);
		glPushMatrix();
		glTranslatef(pos.x, 1, pos.z);
		glScalef(1.3f, 0.1f, 1.3f);
		glutSolidSphere(0.1f, 20, 20);
		glPopMatrix();
	}

	DrawGrid(Physics_Engine::Vector3(20, 1, 22));

	glBegin(GL_LINES);
	glColor3f(0, 0.5, 1);
	for (unsigned i = 0; i < ROD_COUNT; i++)
	{
		Physics_Engine::Particle **particles = rods[i].particle;
		const Physics_Engine::Vector3 &p0 = particles[0]->getPosition();
		const Physics_Engine::Vector3 &p1 = particles[1]->getPosition();
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
	}

	glColor3f(.2, .6, 0);
	for (unsigned i = 0; i < CABLE_COUNT; i++)
	{
		Physics_Engine::Particle **particles = cables[i].particle;
		const Physics_Engine::Vector3 &p0 = particles[0]->getPosition();
		const Physics_Engine::Vector3 &p1 = particles[1]->getPosition();
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
	}

	glColor3f(1, 0.5f, 0.1f);
	for (unsigned i = 0; i < SUPPORT_COUNT; i++)
	{
		const Physics_Engine::Vector3 &p0 = supports[i].particle->getPosition();
		const Physics_Engine::Vector3 &p1 = supports[i].anchor;
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
	}
	glEnd();
}

void BridgeDemo::Update(float duration)
{
	world.startFrame();
	world.runPhysics(duration);
}