#ifndef	PROJECTILE_DEMO_H
#define PROJECTILE_DEMO_H

#include "../Dynamics/particle.h"
#include <GLFW\glfw3.h>
#include <GL\glut.h>

using namespace Physics_Engine;

class ProjectileDemo
{
	enum shotType
	{
		UNUSED = 0,
		PISTOL,
		ARTILLERY,
		FIREBALL,
		LASER
	};

	// Holds a single ammunition round record
	struct AmmoRound
	{
		Physics_Engine::Particle particle;
		shotType type;
		unsigned startTime;

		// Draws the round/projectile
		void render()
		{
			Physics_Engine::Vector3 position;
			particle.getPosition(&position);

			glColor3f(0.1f, 0.1f, 0.5f);
			glPushMatrix();
			glTranslatef(position.x, position.y, position.z);
			glutSolidSphere(0.3f, 5, 4);
			glPopMatrix();

			glColor3f(0.75, 0.75, 0.75);
			glPushMatrix();
			glTranslatef(position.x, 0, position.z);
			glScalef(1.0f, 0.1f, 1.0f);
			glutSolidSphere(0.6f, 5, 4);
			glPopMatrix();
		}
	};

	// Holds the maximum number of rounds that can be fired.
	const static unsigned ammoRounds = 16;						// Why static???? ************************

	// Holds the particle data
	AmmoRound ammo[ammoRounds];

	// Holds the current shot type
	shotType currentShotType;

	// Dispatches a round
	void fire();

public:
	// Creates a new object
	ProjectileDemo();

	// Update the particle positions
	void Update(float duration);

	void KeyInput(GLFWwindow *window);
	bool keyPressed(GLFWwindow *window, int key);

	// Display the particle positions
	void Render();

	void MouseInput(GLFWwindow *window);
};

#endif PROJECTILE_DEMO_H