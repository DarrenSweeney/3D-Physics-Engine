#include "AirplaneDemo.h"
#include <GLFW\glfw3.h>
#include <GL\glut.h>
#include <iostream>

#include "../DebugRender/DebugDrawManager.h"

using namespace Physics_Engine;

GLfloat floorMirror[16] =
{
	1, 0, 0, 0,
	0, -1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1
};

AirplaneDemo::AirplaneDemo() :
right_wing(Matrix3X3(0, 0, 0, -1, -0.5f, 0, 0, 0, 0),
Matrix3X3(0, 0, 0, -0.995f, -0.5f, 0, 0, 0, 0),
Matrix3X3(0, 0, 0, -1.005f, -0.5f, 0, 0, 0, 0),
Vector3(-1.0f, 0.0f, 2.0f), &windspeed),

left_wing(Matrix3X3(0, 0, 0, -1, -0.5f, 0, 0, 0, 0),
Matrix3X3(0, 0, 0, -0.995f, -0.5f, 0, 0, 0, 0),
Matrix3X3(0, 0, 0, -1.005f, -0.5f, 0, 0, 0, 0),
Vector3(-1.0f, 0.0f, -2.0f), &windspeed),

rudder(Matrix3X3(0, 0, 0, 0, 0, 0, 0, 0, 0),
Matrix3X3(0, 0, 0, 0, 0, 0, 0.01f, 0, 0),
Matrix3X3(0, 0, 0, 0, 0, 0, -0.01f, 0, 0),
Vector3(2.0f, 0.5f, 0), &windspeed),

tail(Matrix3X3(0, 0, 0, -1, -0.5f, 0, 0, 0, -0.1f),
Vector3(2.0f, 0, 0), &windspeed),

left_wing_control(0), right_wing_control(0), rudder_control(0),

windspeed(0, 0, 0)
{
	resetPlane();

	aircraft.setMass(2.5f);
	Matrix3X3 inertiaTensor;
	inertiaTensor.setBlockInertiaTensor(Vector3(2, 3, 1), aircraft.getMass());
	aircraft.setInertiaTensor(inertiaTensor);

	aircraft.setDamping(0.8f, 0.8f);

	aircraft.setAcceleration(Vector3(0, -9.81, 0));
	aircraft.calculateDerivedData();

	aircraft.setAwake();
	aircraft.setCanSleep(false);

	registry.add(&aircraft, &left_wing);
	registry.add(&aircraft, &right_wing);
	registry.add(&aircraft, &rudder);
	registry.add(&aircraft, &tail);

	//g_debugDrawManager.addLine(aircraft.getPosition(), Vector3(aircraft.getPosition().x, aircraft.getPosition().y, aircraft.getPosition().z + 10), Vector3(1.0f, 0.0f, 0.0f));
}

AirplaneDemo::~AirplaneDemo()
{

}

void AirplaneDemo::resetPlane()
{
	aircraft.setPosition(0, 0, 0);
	aircraft.setOrientation(1, 0, 0, 0);
	aircraft.setVelocity(0, 0, 0);
	aircraft.setRotation(0, 0, 0);

	left_wing_control = 0;
	right_wing_control = 0;
	rudder_control = 0;
}

void AirplaneDemo::Update(float duration)
{
	if (duration <= 0.0)
		return;

	aircraft.clearAccumulators();

	Vector3 propulsion(-7.0f, 0.0f, 0.0f);
	propulsion = aircraft.getTransform().transformDirection(propulsion);
	if (aircraft.getVelocity().magnitude() <= 20)
		aircraft.addForce(propulsion);

	registry.updateForces(duration);

	aircraft.integrate(duration);

	Vector3 pos = aircraft.getPosition();
	if (pos.y < 0.0f)
	{
		pos.y = 0.0f;
		aircraft.setPosition(pos);

		if (aircraft.getVelocity().y < -10.0f)
		{
			resetPlane();
		}
	}
}

void AirplaneDemo::KeyInput(GLFWwindow *window)
{
	// Rudder controls.
	if (keyPressed(window, GLFW_KEY_RIGHT))
	{
		rudder_control -= 0.1f;
	}
	else if (keyPressed(window, GLFW_KEY_LEFT))
	{
		rudder_control += 0.1f;
	}
	else
		rudder_control = 0;

	// Wing Control.
	if (keyPressed(window, GLFW_KEY_W))
	{
		left_wing_control -= 0.1f;
		right_wing_control -= 0.1f;
	}
	else if (keyPressed(window, GLFW_KEY_S))
	{
		left_wing_control += 0.5f;
		right_wing_control += 0.5f;
	}
	if (keyPressed(window, GLFW_KEY_A))
	{
		left_wing_control += 0.1f;
		right_wing_control -= 0.1f;
	}
	else if (keyPressed(window, GLFW_KEY_D))
	{
		left_wing_control -= 0.1f;
		right_wing_control += 0.1f;
	}
	else
	{
		left_wing_control = 0;
		right_wing_control = 0;
	}

	// Make sure the controls are in range
	if (left_wing_control < -1.0f)
		left_wing_control = -1.0f;
	else if (left_wing_control > 3.0f)
		left_wing_control = 3.0f;
	if (right_wing_control < -1.0f)
		right_wing_control = -1.0f;
	else if (right_wing_control > 1.0f)
		right_wing_control = 1.0f;
	if (rudder_control < -1.0f)
		rudder_control = -1.0f;
	else if (rudder_control > 1.0f)
		rudder_control = 1.0f;

	// Update the control surfaces
	left_wing.setControl(left_wing_control);
	right_wing.setControl(right_wing_control);
	rudder.setControl(rudder_control);
}

bool AirplaneDemo::keyPressed(GLFWwindow *window, int key)
{
	int state = glfwGetKey(window, key);
	if (state == GLFW_PRESS)
		return true;
	else
		return false;
}

void AirplaneDemo::drawAircarft()
{
	// Fuselage
	glColor3f(0.01f, 0.52f, 0.5f);
	glPushMatrix();
	glTranslatef(-0.5f, 0, 0);
	glScalef(2.0f, 0.8f, 1.0f);
	glutSolidCube(1.0f);
	glPopMatrix();

	// Rear Fuselage
	glColor3f(0.01f, 0.02f, 0.5f);
	glPushMatrix();
	glTranslatef(1.0f, 0.15f, 0);
	glScalef(2.75f, 0.5f, 0.5f);
	glutSolidCube(1.0f);
	glPopMatrix();

	// Wings
	glColor3f(0.4f, 0.02f, 0.5f);
	glPushMatrix();
	glTranslatef(0, 0.3f, 0);
	glScalef(0.8f, 0.1f, 6.0f);
	glutSolidCube(1.0f);
	glPopMatrix();

	// Rudder
	glColor3f(0.01f, 0.42f, 0.5f);
	glPushMatrix();
	glTranslatef(2.0f, 0.775f, 0);
	glScalef(0.75f, 1.15f, 0.1f);
	glutSolidCube(1.0f);
	glPopMatrix();

	// Tail-plane
	glColor3f(0.21f, 0.02f, 0.5f);
	glPushMatrix();
	glTranslatef(1.9f, 0, 0);
	glScalef(0.85f, 0.1f, 2.0f);
	glutSolidCube(1.0f);
	glPopMatrix();

	//g_debugDrawManager.AddAxes(aircraft.getTransform(), 4.0f);
	//g_debugDrawManager.AddCross(aircraft.getPointInLocalSpace(aircraft.getPosition()), Vector3(1, 0, 1), 4.0f);
}

void AirplaneDemo::DrawGrid(Vector3 &pos)
{
	// Drawing a Grid Plane
	glBegin(GL_LINES);
	unsigned width = 1000;
	for (unsigned i = 0; i <= width; i += 10)
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

void AirplaneDemo::Render()
{
	Vector3 pos = aircraft.getPosition();
	Vector3 offset(4.0f + aircraft.getVelocity().magnitude(), 0, 0);
	offset = aircraft.getTransform().transformDirection(offset);

	gluLookAt(pos.x + offset.x, pos.y + 5.0f, pos.z + offset.z,
		pos.x, pos.y, pos.z,
		0.0, 1.0, 0.0);

	glColor3f(0.3f, 0.6f, 0.6f);
	int bx = int(pos.x);
	int bz = int(pos.z);

	Matrix3X4 transform = aircraft.getTransform();
	GLfloat gl_transform[16];
	transform.fillGLArray(gl_transform);

	glPushMatrix();
	glMultMatrixf(gl_transform);
	drawAircarft();
	glPopMatrix();

	DrawGrid(Vector3(500, 0, 500));
}