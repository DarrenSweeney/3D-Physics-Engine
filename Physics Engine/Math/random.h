#ifndef RANDOM_H
#define RANDOM_H

#include "core.h"

namespace Physics_Engine
{
	class Random
	{
	public:
		Random();

		float randomFloat(const float minNum, const float maxNum);

		int randomInt(const int minNum, const int maxNum);

		real randomReal(const real minNum, const real maxNum);

		Vector3 randomVec(const Vector3 &minVec, const Vector3 &maxVec);
	};
}

#endif // RANDOM_H