#include "../Math/core.h"
#include "../Dynamics/force_gen.h"
#include <GLFW\glfw3.h>

using namespace Physics_Engine;

class AirplaneDemo
{
	AeroControl left_wing;
	AeroControl right_wing;
	AeroControl rudder;
	Aero tail;
	RigidBody aircraft;
	ForceRegistry registry;

	Vector3 windspeed;

	float left_wing_control;
	float right_wing_control;
	float rudder_control;

	void drawAircarft();

	void DrawGrid(Vector3 &position);

public:
	AirplaneDemo();
	~AirplaneDemo();

	void Update(float duration);	
	void Render();

	void resetPlane();
	void KeyInput(GLFWwindow *window);
	bool keyPressed(GLFWwindow *window, int keyID);
};