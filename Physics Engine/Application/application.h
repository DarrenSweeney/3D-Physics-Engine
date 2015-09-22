#include "Timer.h"

class Application
{
protected:
	// Holds the width and height of the application window
	unsigned int width, height;

	Physics_Engine::Timer timer;

	bool switchDemos[5];

	// Drawing visual information of the physics simulation.
	bool debugDrawMode = false;

public:
	int Execute();

private:
	bool isRunning;

protected:
	virtual void Init();
	void SetView();
	virtual void Update();
	virtual void Render();
	virtual void CleanUp();

	struct GLFWwindow *getWindow();

	virtual bool Key(int key);

	bool getDemo(unsigned int index);
};