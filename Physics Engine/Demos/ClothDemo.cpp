#include "ClothDemo.h"
#include "../Math/core.h"
#include "../Dynamics/particle.h"
#include "../Dynamics/constraint.h"
#include "../Dynamics/cloth.h"
#include <iostream>

#include <GL\glut.h>

Cloth  cloth(10, 10, 50, 50);
Sphere sphere(Vector3(7, -5, 0), 1.5f);

void ClothDemo::Update(float duration)
{
	if (duration <= 0.0f)
		return;

	cloth.addForce(Vector3(0.0, -0.02, 0.0));
	cloth.addWindForce(Vector3(0.5, 0, 0.2));
	cloth.timeStep(duration);
	cloth.ballCollision(sphere);
}

void ClothDemo::KeyInput(GLFWwindow *window)
{
	if (keyPressed(window, GLFW_KEY_W))
		sphere.pos.z -= 0.2f * (0.5f * 0.5f);
	if (keyPressed(window, GLFW_KEY_S))
		sphere.pos.z += 0.2f * (0.5f * 0.5f);
	if (keyPressed(window, GLFW_KEY_A))
		sphere.pos.x -= 0.2f * (0.5f * 0.5f);
	if (keyPressed(window, GLFW_KEY_D))
		sphere.pos.x += 0.2f * (0.5f * 0.5f);
}

bool ClothDemo::keyPressed(GLFWwindow *window, int key)
{
	int state = glfwGetKey(window, key);
	if (state == GLFW_PRESS)
	{
		return true;
	}
	else
		return false;
}

void ClothDemo::DrawGrid(Vector3 pos)
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

void ClothDemo::DebugRender()
{
	cloth.debugDraw();
}

void ClothDemo::Render()
{
	DrawGrid(Vector3(20, 0, 20));

	glTranslatef(-2, 8, -2);
	glRotatef(45, 0, 1, 0);

	cloth.draw();

	const static GLfloat lightPosition[] = { 0, 3, 8, 0 };

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glPushMatrix();
	glTranslatef(sphere.pos.x, sphere.pos.y, sphere.pos.z);
	glColor3f(0.9f, 1.0f, 0.0f);
	glutSolidSphere(sphere.radius - 0.1, 50, 50);
	glPopMatrix();
	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
}