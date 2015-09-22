#ifndef DEBUG_DRAWING_MANAGER_H
#define DEBUG_DRAWING_MANAGER_H

#include "../Math/core.h"
#include <GLFW\glfw3.h>

class DebugDrawManager
{
public:
	void AddLine(const Physics_Engine::Vector3 &point,
		const Physics_Engine::Vector3 &endPoint,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real lineWidth = 0.01f,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddCross(const Physics_Engine::Vector3 &point,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real size,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddSphere(const Physics_Engine::Vector3 &centerPosition,
		Physics_Engine::real radius,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddCircle(const Physics_Engine::Vector3 &centerPosition,
		const Physics_Engine::Vector3 &planeNormal,
		Physics_Engine::real radius,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddAxes(const Physics_Engine::Matrix3X4 &transform,
		Physics_Engine::real size,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddTriangle(const Physics_Engine::Vector3 &vertexPos_1,
		const Physics_Engine::Vector3 &vertexPos_2,
		const Physics_Engine::Vector3 &vertexPos_3,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real lineWidth = 0.01f,
		Physics_Engine::real dureation = 0.0f,
		bool depthEnabled = true);

	void AddAABB(const Physics_Engine::Vector3 &minCoords,
		const Physics_Engine::Vector3 &maxCoords,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real lineWidth = 0.01f,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);

	void AddOBB(const Physics_Engine::Matrix3X4 &transformCenter,
		const Physics_Engine::Vector3 &scaleXYZ,
		Physics_Engine::Vector3 &color,
		Physics_Engine::real lineWidth = 0.01f,
		Physics_Engine::real duration = 0.0f,
		bool depthEnabled = true);
};

extern DebugDrawManager g_debugDrawManager;

#endif DEBUG_DRAWING_MANAGER_H