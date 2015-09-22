#include <iostream>
#include "ProjectileDemo.h"

ProjectileDemo::ProjectileDemo()
: currentShotType(ARTILLERY)
{
	// Make all shots unused 
	for (AmmoRound *shot = ammo; shot < ammo + ammoRounds; shot++)
	{
		shot->type = UNUSED;
	}
}

void ProjectileDemo::fire()
{
	// Find the first available round.
	AmmoRound *shot;

	for (shot = ammo; shot < ammo + ammoRounds; shot++)
	{
		if (shot->type == UNUSED)
		{
			break;
		}
	}

	// If we didn't find a round, then exit - we can't fire
	if (shot >= ammo + ammoRounds)
	{
		return;
	}

	// Set the properties of the particle	---SI units---
	switch (currentShotType)
	{
	case PISTOL:
		shot->particle.setMass(2.0f); // 2.0kg
		shot->particle.setVelocity(0.0f, 0.0f, 35.0f); // 35m/s
		shot->particle.setAcceleration(0.0f, -1.0f, 0.0f);
		shot->particle.setDamping(0.99f);
		break;

	case ARTILLERY:
		shot->particle.setMass(200.0f); // 200.0kg
		shot->particle.setVelocity(-3.0f, 25.0f, 20.0f); // 50m/s
		shot->particle.setAcceleration(-1.0f, -20.0f, 0.0f);
		shot->particle.setDamping(0.99f);
		break;

	case FIREBALL:
		shot->particle.setMass(1.0f); // 1.0kg - mostly blast damage
		shot->particle.setVelocity(0.0f, 0.0f, 10.0f); // 5m/s
		shot->particle.setAcceleration(0.0f, 0.9f, 0.0f); // Floats up
		shot->particle.setDamping(0.9f);
		break;

	case LASER:
		shot->particle.setMass(0.1f); // 0.1kg - almost no weight
		shot->particle.setVelocity(0.0f, 0.0f, 100.0f); // 100m/s
		shot->particle.setAcceleration(0.0f, 0.0f, 0.0f); // No gravity
		shot->particle.setDamping(0.99f);
		break;
	}

	// Set the data common to all particles types
	shot->particle.setPosition(0.0f, 1.5f, 0.0f);
	shot->type = currentShotType;

	shot->particle.clearAccumulator();
}

void ProjectileDemo::Update(float duration)
{
	if (duration <= 0.0f)
		return;

	// Update the physics of each particle in turn
	for (AmmoRound *shot = ammo; shot < ammo + ammoRounds; shot++)
	{
		if (shot->type != UNUSED)
		{
			// Run the physics
			shot->particle.intergrate(duration);

			// Check to see if the particle is now invalid
			if (shot->particle.getPosition().y < 0.0f ||
				shot->particle.getPosition().y > 50.0f ||
				shot->particle.getPosition().z > 200.0f)
			{
				shot->type = UNUSED;
			}
		}
	}
}

void ProjectileDemo::KeyInput(GLFWwindow *window)
{
	if (keyPressed(window, GLFW_KEY_1))
		currentShotType = shotType::ARTILLERY;

	if (keyPressed(window, GLFW_KEY_2))
		currentShotType = shotType::PISTOL;

	if (keyPressed(window, GLFW_KEY_3))
		currentShotType = shotType::LASER;

	if (keyPressed(window, GLFW_KEY_4))
		currentShotType = shotType::FIREBALL;
}

bool ProjectileDemo::keyPressed(GLFWwindow *window, int key)
{
	int state = glfwGetKey(window, key);
	if (state == GLFW_PRESS)
	{
		return true;
	}
	else
		return false;
}

int lastState;
void ProjectileDemo::MouseInput(GLFWwindow *window)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if (state == GLFW_PRESS && lastState == GLFW_RELEASE)
		fire();

	lastState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
}

void ProjectileDemo::Render()
{
	// Clear the viewport
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(-25.0, 8.0, 5.0, 0.0, 5.0, 22.0, 0.0, 1.0, 0.0);

	// Draw a sphere at the firing point and add a shadow projected
	// onto the ground plane
	glColor3f(0.0f, 0.0f, 0.0f);
	glPushMatrix();
	glTranslatef(0.0f, 1.5f, 0.0f);
	glutSolidSphere(0.1f, 5, 5);
	glTranslatef(0.0f, -1.5f, 0.0f);
	glColor3f(0.75f, 0.75f, 0.75f);
	glScalef(1.0f, 0.1f, 1.0f);
	glutSolidSphere(0.1f, 5, 5);
	glPopMatrix();

	// Draw some cool scale lines
	glColor3f(0.75f, 0.75f, 0.75f);
	glBegin(GL_LINES);
	for (unsigned int i = 0; i < 200; i += 10)
	{
		glVertex3f(-5.0f, 0.0f, i);
		glVertex3f(5.0f, 0.0f, i);
	}
	glVertex3f(0.0f, 0.0f, -1.0f);
	glVertex3f(0.0f, 0.0f, 200.0f);
	glEnd();

	// Render each particle in turn
	for (AmmoRound *shot = ammo; shot < ammo + ammoRounds; shot++)
	{
		if (shot->type != UNUSED)
		{
			shot->render();
		}
	}
}