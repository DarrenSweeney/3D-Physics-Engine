#include "../Application/application.h"
#include "../Demos/ClothDemo.h"
#include "../Demos/AirplaneDemo.h"
#include "../Demos/ProjectileDemo.h"
#include "../Demos/BridgeDemo.h"
#include "../Demos/CollisionTest.h"
#include <GLFW\glfw3.h>

class PhysicsEngineDemo : public Application
{
public:
	enum Demos
	{
		cloth,
		airplane,
		projectile,
		bridge,
		swingRope,
		collisionTest
	};
	
	Demos demos = Demos::cloth;

private:
	ClothDemo clothDemo;
	AirplaneDemo airplaneDemo;
	ProjectileDemo projectileDemo;
	BridgeDemo bridgeDemo;
	CollisionTest collisionTestDemo;

public:
	virtual void Update();
	virtual void Render();
	void setDemoState(Demos demo);
};