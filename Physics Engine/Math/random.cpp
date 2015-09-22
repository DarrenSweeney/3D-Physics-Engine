#include <cstdlib>
#include <ctime>
#include "random.h"

using namespace Physics_Engine;

Random::Random()
{
	srand(static_cast<unsigned int>(time(0)));
}

float Random::randomFloat(const float minNum, const float maxNum)
{
	return rand() / ((float)(RAND_MAX) + 1);
}

int Random::randomInt(const int minNum, const int maxNum)
{
	if (maxNum == 0)
		return 0;
	return (rand() % maxNum) + minNum;
}

real Random::randomReal(const real minNum, const real maxNum)
{
	return rand() * (maxNum - minNum) + minNum;
}

Vector3 Random::randomVec(const Vector3 &minVec, const Vector3 &maxVec)
{
	return Vector3(randomInt(minVec.x, maxVec.x), randomInt(minVec.y, maxVec.y), randomInt(minVec.z, maxVec.z));
}