#ifndef TIMER_H
#define TIMER_H

namespace Physics_Engine
{
	class Timer
	{
	private:
		double currentTime, previousTime;
		float duration;

		bool timerPaused;
		bool timerStarted;

	public:
		Timer();

		void Update();

		bool isStarted() const;
		bool isPaused() const;

		float getTicks();
	};
}
#endif