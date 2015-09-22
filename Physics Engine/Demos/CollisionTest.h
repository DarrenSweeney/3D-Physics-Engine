#ifndef COLLISION_TEST
#define COLLISION_TEST

#include "../Collision/NarrowPhase.h"
#include <GLFW\glfw3.h>
#include "../Math/random.h"

namespace Physics_Engine
{

#define OBJECTS 30;

	class Ball : public CollisionSphere
	{
	public:
		Ball();
		~Ball();

		void render();
		void renderShadow();
		void setState(Vector3 &position, Quaternion &orientation, real radius, Vector3 &velocity);
	};

	class Box : public CollisionBox
	{
	public:
		Box();
		~Box();

		bool isOverlapping;

		void render();
		void renderShadow();
		void setState(Vector3 &position, Quaternion &orientation, Vector3 &extents, Vector3 &velocity);
	};

	class CollisionTest
	{
	public:
		/*
			Holds a transform matrix for rendering objects
			reflecting in the floor.
		*/
		GLfloat floorMirror[16] =
		{
			1, 0, 0, 0,
			0,-1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1
		};

		const static unsigned maxContacts = 256;
		Contact contacts[maxContacts];
		CollisionData collisionData;
		ContactResolver resolver;
		
		// Holds the camera angle.
		float theta;
		// Holds the camera elevation.
		float phi;
		bool renderDebugInfo;
		bool pauseSimulation;
		bool autoPauseSimulation;
		void drawDebug();

	private:
		const static unsigned boxes = OBJECTS;
		Box boxData[boxes];

		const static unsigned balls = OBJECTS;
		Ball ballData[balls];

		Random random;

	public:
		void reset();
		void generateContacts();
		void updateObjects(real duration);

		CollisionTest();
		void initGraphics();
		void display();
		void key(GLFWwindow *window);
	};
}
#endif;