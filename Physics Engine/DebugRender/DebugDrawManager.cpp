#include "DebugDrawManager.h"
#include <GL\glut.h>

DebugDrawManager g_debugDrawManager;

void DebugDrawManager::AddLine(const Physics_Engine::Vector3 &point,
	const Physics_Engine::Vector3 &endPoint,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real lineWidth,
	Physics_Engine::real duration,
	bool depthEnabled)
{
	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINES); 
	glVertex3f(point.x, point.y, point.z);
	glVertex3f(endPoint.x + lineWidth, endPoint.y + lineWidth, endPoint.z + lineWidth);
	glEnd();
}

void DebugDrawManager::AddCross(const Physics_Engine::Vector3 &point,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real size,
	Physics_Engine::real duration,
	bool depthEnabled)
{
	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINES);
	// Line X
	glVertex3f(point.x - size, point.y, point.z);
	glVertex3f(point.x + size, point.y, point.z);

	// Line Y
	glVertex3f(point.x, point.y - size, point.z);
	glVertex3f(point.x, point.y + size, point.z);

	// Line z
	glVertex3f(point.x, point.y, point.z - size);
	glVertex3f(point.x, point.y, point.z + size);
	glEnd();
}

void DebugDrawManager::AddSphere(const Physics_Engine::Vector3 &centerPosition,
	Physics_Engine::real radius,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real duration,
	bool depthEnabled)
{
	glPushMatrix();
	glTranslatef(centerPosition.x, centerPosition.y, centerPosition.z);
	glColor3f(color.x, color.y, color.z);
	glutWireSphere(radius, 30, 30);
	glPopMatrix();
}

void DebugDrawManager::AddCircle(const Physics_Engine::Vector3 &centerPosition,
	const Physics_Engine::Vector3 &planeNormal,
	Physics_Engine::real radius,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real duration,
	bool depthEnabled)
{
	glColor3f(color.x, color.y, color.z);
	glBegin(GL_LINE_LOOP);
	const int NUM_OF_SEGMENTS = 20;
	for (int i = 0; i < NUM_OF_SEGMENTS; i++)
	{
		float theta = 2.0f * 3.1415926f * float(i) / float(NUM_OF_SEGMENTS); // Get the current angle.

		float x = radius * cosf(theta); // Calculate the x component.
		float y = radius * sinf(theta); // Calculate the y component.

		glVertex2f(x + centerPosition.x, y + centerPosition.y); // Output vertex.

	}
	glEnd();
}

void DebugDrawManager::AddTriangle(const Physics_Engine::Vector3 &vertexPos_1,
	const Physics_Engine::Vector3 &vertexPos_2,
	const Physics_Engine::Vector3 &vertexPos_3,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real lineWidth,
	Physics_Engine::real dureation,
	bool depthEnabled)
{
	glBegin(GL_TRIANGLES);
	glColor3f(color.x, color.y, color.z);
	glVertex3f(vertexPos_1.x, vertexPos_1.y, vertexPos_1.z);
	glVertex3f(vertexPos_2.x, vertexPos_2.y, vertexPos_2.z);
	glVertex3f(vertexPos_3.x, vertexPos_3.y, vertexPos_3.z);
	glEnd();
}

void DebugDrawManager::AddAABB(const Physics_Engine::Vector3 &minCoords,
	const Physics_Engine::Vector3 &maxCoords,
	Physics_Engine::Vector3 &color,
	Physics_Engine::real lineWidth,
	Physics_Engine::real duration,
	bool depthEnabled)
{
	glColor3f(color.x, color.y, color.z);

	// Top
	glBegin(GL_LINES);
	glVertex3f(minCoords.x, minCoords.y, minCoords.z);
	glVertex3f(maxCoords.x, minCoords.y, minCoords.z);

	glVertex3f(maxCoords.x, minCoords.y, minCoords.z);
	glVertex3f(maxCoords.x, minCoords.y,maxCoords.z);

	glVertex3f(maxCoords.x, minCoords.y, maxCoords.z);
	glVertex3f(minCoords.x, minCoords.y, maxCoords.z);

	glVertex3f(minCoords.x, minCoords.y, maxCoords.z);
	glVertex3f(minCoords.x, minCoords.y, minCoords.z);
	glEnd();

	// Bottom
	glBegin(GL_LINES);
	glVertex3f(minCoords.x, maxCoords.y, minCoords.z);
	glVertex3f(maxCoords.x, maxCoords.y, minCoords.z);

	glVertex3f(maxCoords.x, maxCoords.y, minCoords.z);
	glVertex3f(maxCoords.x, maxCoords.y, maxCoords.z);

	glVertex3f(maxCoords.x, maxCoords.y, maxCoords.z);
	glVertex3f(minCoords.x, maxCoords.y, maxCoords.z);

	glVertex3f(minCoords.x, maxCoords.y, maxCoords.z);
	glVertex3f(minCoords.x, maxCoords.y, minCoords.z);
	glEnd();

	// Back
	glBegin(GL_LINES);
	glVertex3f(minCoords.x, minCoords.y, minCoords.z);
	glVertex3f(minCoords.x, maxCoords.y, minCoords.z);

	glVertex3f(maxCoords.x, minCoords.y, minCoords.z);
	glVertex3f(maxCoords.x, maxCoords.y, minCoords.z);
	glEnd();

	// Front
	glBegin(GL_LINES);
	glVertex3f(minCoords.x, minCoords.y, maxCoords.z);
	glVertex3f(minCoords.x, maxCoords.y, maxCoords.z);

	glVertex3f(maxCoords.x, minCoords.y, maxCoords.z);
	glVertex3f(maxCoords.x, maxCoords.y, maxCoords.z);
	glEnd();
}