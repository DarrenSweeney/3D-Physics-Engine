#ifndef CLOTHDEMO_H
#define CLOTHDEMO_H

#include "../Math/core.h"
#include <GLFW\glfw3.h>

using namespace Physics_Engine;

class ClothDemo
{

public:
	void Update(float duration);
	void Render();
	void DebugRender();
	void DrawGrid(Vector3 pos);

	void KeyInput(GLFWwindow *window);
	bool keyPressed(GLFWwindow *window, int keyID);
};

#endif CLOTHDEMO_H