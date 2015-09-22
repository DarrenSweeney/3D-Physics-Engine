#include "Timer.h"
#include <GLFW\glfw3.h>

using namespace Physics_Engine;

Timer::Timer()
{
	currentTime = 0;
	previousTime = 0;

	timerStarted = true;
	timerPaused = false;
}

void Timer::Update()
{
	if (timerPaused == false)
	{
		previousTime = currentTime;
		currentTime = glfwGetTime();
		duration = (currentTime - previousTime);
	}
}

float Timer::getTicks()
{
	return duration;
}

bool Timer::isStarted() const
{
	return timerStarted;
}

bool Timer::isPaused() const
{
	return timerPaused;
}