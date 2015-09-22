#include "Physics_Engine_Demo.h"
#include "../Imgui/imgui.h"

void PhysicsEngineDemo::Update()
{
	Application::Update();

	timer.Update();
	float duration = timer.getTicks();

	if (getDemo(0) && demos != Demos::airplane)
	{
		demos = Demos::airplane;
		airplaneDemo.resetPlane();
	}
	if (getDemo(1) && demos != Demos::cloth)
		demos = Demos::cloth;
	if (getDemo(2) && demos != Demos::projectile)
		demos = Demos::projectile;
	if (getDemo(3) && demos != Demos::bridge)
	{
		demos = Demos::bridge;
		bridgeDemo.ResetDemo();
	}
	if (getDemo(4) && demos != Demos::collisionTest)
	{
		demos = Demos::collisionTest;
		collisionTestDemo.reset();
	}
	
	switch (demos)
	{
	case cloth:
		clothDemo.Update(duration);
		clothDemo.KeyInput(getWindow());
		break;
	case airplane:
		airplaneDemo.Update(duration);
		airplaneDemo.KeyInput(getWindow());
		break;
	case projectile:
		projectileDemo.Update(duration);
		projectileDemo.KeyInput(getWindow());
		projectileDemo.MouseInput(getWindow());
		break;
	case bridge:
		bridgeDemo.Update(duration);
		break;
	case collisionTest:
		collisionTestDemo.updateObjects(duration);
		collisionTestDemo.key(getWindow());
		break;

	default:
		break;
	}
}

void PhysicsEngineDemo::setDemoState(Demos demo)
{
	demos = demo;
}

void PhysicsEngineDemo::Render()
{
	Application::Render();

	switch (demos)
	{
	case cloth:
		clothDemo.Render();
		if (debugDrawMode)
			clothDemo.DebugRender();
		break;
	case airplane:
		airplaneDemo.Render();
		break;
	case projectile:
		projectileDemo.Render();
		break;
	case bridge:
		bridgeDemo.Render();
		break;
	case collisionTest:
		collisionTestDemo.display();
		break;
	default:
		break;
	}
}

Application *getApplication()
{
	return new PhysicsEngineDemo();
} 