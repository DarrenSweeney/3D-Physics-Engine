#include "CollisionTest.h"
#include <GL\glut.h>

using namespace Physics_Engine;

Ball::Ball()
{
	body = new RigidBody();
}

Ball::~Ball()
{
	delete body;
}

void Ball::render()
{
	// Get the OpenGL transformation.
	GLfloat mat[16];
	body->getGLTransform(mat);

	if (body->getAwake() == true)
		glColor3f(1.0f, 0.7f, 0.7f);
	else
		glColor3f(0.0f, 0.0f, 0.0f);

	glColor3f(1, .3f, 0);
	glPushMatrix();
	glMultMatrixf(mat);
	glutSolidSphere(radius, 20, 20);
	glPopMatrix();
}

void Ball::renderShadow()
{
	// Get the OpenGL transformation.
	GLfloat mat[16];
	body->getGLTransform(mat);

	glPushMatrix();
	glScalef(1.0f, 0, 1.0f);
	glMultMatrixf(mat);
	glutSolidSphere(radius, 20, 20);
	glPopMatrix();
}

void Ball::setState(Vector3 &position, Quaternion &orientation, real radius, Vector3 &velocity)
{
	body->setPosition(position);
	body->setOrientation(orientation);
	body->setVelocity(velocity);
	body->setRotation(Vector3());
	Ball::radius = radius;

	real mass = 4.0f * 0.3333f * 3.1415f * radius * radius * radius;
	body->setMass(mass);

	Matrix3X3 tensor;
	real coeff = 0.4f*mass*radius*radius;
	tensor.setInertiaTensorCoeffs(coeff, coeff, coeff);
	body->setInertiaTensor(tensor);

	body->setLinearDamping(0.95f);
	body->setAngularDamping(0.8f);
	body->clearAccumulators();
	body->setAcceleration(0, -10.0f, 0);

	body->setAwake(true);

	body->calculateDerivedData();
}


Box::Box()
{
	body = new RigidBody();
}

Box::~Box()
{
	delete body;
}

void Box::render()
{
	// Get the OpenGL transformation.
	GLfloat mat[16];
	body->getGLTransform(mat);

	if (isOverlapping)
		glColor3f(0.7f, 1.0f, 0.7f);
	
	if (body->getAwake() == true)
		glColor3f(0.0f, 0.2f, 0.8f);
	else
		glColor3f(0.0f, 0.2f, 0.8f);

	//glColor3f(0.0f, 0.2f, 0.8f);
	glPushMatrix();
	glMultMatrixf(mat);
	glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
	glutSolidCube(1.0f);
	glPopMatrix();
}

void Box::renderShadow()
{
	GLfloat mat[16];
	body->getGLTransform(mat);

	glPushMatrix();
	glScalef(1.0f, 0, 1.0f);
	glMultMatrixf(mat);
	glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
	glutSolidCube(1.0f);
	glPopMatrix();
}

void Box::setState(Vector3 &position, Quaternion &orientation, Vector3 &extents, Vector3 &velocity)
{
	body->setPosition(position);
	body->setOrientation(orientation);
	body->setVelocity(velocity);
	body->setRotation(Vector3());
	halfSize = extents;

	real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
	body->setMass(mass);

	Matrix3X3 tensor;
	tensor.setBlockInertiaTensor(halfSize, mass);
	body->setInertiaTensor(tensor);

	body->setLinearDamping(0.95f);
	body->setAngularDamping(0.8f);
	body->clearAccumulators();
	body->setAcceleration(0, -10.0f, 0);

	body->setCanSleep(true);
	body->setAwake();

	body->calculateDerivedData();
}


CollisionTest::CollisionTest()
	:
	theta(0.0f),
	phi(15.0f),
	resolver(10),
	random(),
	renderDebugInfo(true),
	pauseSimulation(true),
	autoPauseSimulation(false)
{
	collisionData.contactArray = contacts;
	reset();
}

void CollisionTest::reset()
{
	// Create the objects.
	for (Box *box = boxData; box < boxData + boxes; box++)
	{

		box->setState(random.randomVec(Vector3(-13, 2, -11), Vector3(10, 20, -30)), Quaternion(1, 0, 0, 0), Vector3(1, 1, 1), Vector3(0, 0, 0));
	}

	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		//ball->setState(random.randomVec(Vector3(-10, 0, -10), Vector3(5, 10, 15)), Quaternion(1, 0, 0, 0), 1.0f, Vector3(0, 0, 0));
	}

	// Reset the contacts.
	collisionData.contactCount = 0;
}

void CollisionTest::generateContacts()
{
	/*
		Note that this method makes a lot of use of early returns to avoid
		processing lots of potential contacts that it hasn't got room to store.
	*/
	CollisionPlane plane;
	plane.normal = Vector3(0, 1, 0);
	plane.offset = 0;

	// Set up the collision data structure.
	collisionData.reset(maxContacts);
	collisionData.friction = (real)0.9;
	collisionData.restitution = (real)0.6;
	collisionData.tolerance = (real)0.1;

	// Perform exhaustive collision dectection
	Matrix3X4 transform, otherTransform;
	Vector3 position, otherPosition;

	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		// Check for collisions with the ground plane.
		if (!collisionData.hasMoreContacts())
			return;

		CollisionDectector::boxAndHalfSpace(*box, plane, &collisionData);

		// Check for collisions with each other box.
		for (Box *other = box + 1; other < boxData + boxes; other++)
		{
			if (!collisionData.hasMoreContacts())
				return;

			CollisionDectector::boxAndBox(*box, *other, &collisionData);

			if (IntersectionTests::boxAndBox(*box, *other))
			{
				box->isOverlapping = other->isOverlapping = true;
			}
		}

		// Check for collisions with each ball.
		for (Ball *other = ballData; other < ballData + balls; other++)
		{
			if (!collisionData.hasMoreContacts())
				return;

			CollisionDectector::boxAndSphere(*box, *other, &collisionData);
		}
	}

	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		// Check for collisions with the ground plane.
		if (!collisionData.hasMoreContacts())
			return;

		CollisionDectector::sphereAndHalfSpace(*ball, plane, &collisionData);

		for (Ball *other = ballData + 1; other < ballData + balls; other++)
		{
			if (!collisionData.hasMoreContacts())
				return;

			CollisionDectector::sphereAndSphere(*ball, *other, &collisionData);
		}
	}
}

void CollisionTest::updateObjects(real duration)
{
	generateContacts();

	resolver.resolveContacts(collisionData.contactArray, collisionData.contactCount, duration);

	// Update the physics of each box in turn.
	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		// Run the physics.
		box->body->integrate(duration);
		box->calculateInternals();
		box->isOverlapping = false;
	}

	// Update the physics of each ball in turn.
	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		// Run the physics.
		//ball->body->integrate(duration);
		//ball->calculateInternals();
	}
}

void CollisionTest::key(GLFWwindow *window)
{
	int state = glfwGetKey(window, GLFW_KEY_R);
	if (state == GLFW_PRESS)
	{
		reset();
	}
}
void CollisionTest::initGraphics()
{
	GLfloat lightAmbient[] = {0.8f, 0.8f, 0.8f, 1.0f};
	GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.0f, 1.0f};

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

	glEnable(GL_LIGHT0);
}

void DrawGrid(Vector3 pos, unsigned w)
{
	// Drawing a Grid Plane
	glBegin(GL_LINES);
	unsigned width = w;
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

void CollisionTest::display()
{
	const static GLfloat lightPosition[] = {1, -1, 0, 0};
	const static GLfloat lightPositionMirror[] = { 1, 1, 0, 0 };

	// Update the transform matrices of each box in turn.
	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		box->calculateInternals();
		box->isOverlapping = false;
	}

	// Update the transform matrices of each ball in turn.
	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		// Run the physics.
		ball->calculateInternals();
	}

	// Clear the viewport and set the camera direction.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(18.0f, 0, 0, 0, 0, 0, 0, 1.0f, 0);
	glRotatef(-phi, 0, 0, 1);
	glRotatef(theta, 0, 1, 0);
	glTranslatef(0, -5.0f, 0);

	// Render each element in turn as a shadow.
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glEnable(GL_LIGHT0);
	glPushMatrix();
	glMultMatrixf(floorMirror);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		box->render();
	}
	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		ball->render();
	}
	glPopMatrix();
	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);

	DrawGrid(Vector3(35, 0, 30), 60);

	// Render each shadow in turn
	glEnable(GL_BLEND);
	glColor4f(0, 0, 0, 0.1f);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		box->renderShadow();
	}
	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		ball->renderShadow();
	}
	glDisable(GL_BLEND);
	
	// Render the boxes themselves
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPositionMirror);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	for (Box *box = boxData; box < boxData + boxes; box++)
	{
		box->render();
	}
	for (Ball *ball = ballData; ball < ballData + balls; ball++)
	{
		ball->render();
	}
	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	
	drawDebug();
}

void CollisionTest::drawDebug()
{
	if (!renderDebugInfo)
		return;

	/*
		Recalculate the contacts, so they are current.
		(In case we are paused, for example).
	*/
	generateContacts();

	// Render the contacts, if required.
	glBegin(GL_LINES);
	for (unsigned i = 0; i < collisionData.contactCount; i++)
	{
		// Interbody contacts are in green, floor contacts are red.
		if (contacts[i].body[1])
			glColor3f(0, 1, 0);
		else
			glColor3f(1, 0, 0);

		Vector3 vec = contacts[i].contactPoint;
		glVertex3f(vec.x, vec.y, vec.z);

		vec += contacts[i].contactNormal;
		glVertex3f(vec.x, vec.y, vec.z);
	}
	glEnd();
}