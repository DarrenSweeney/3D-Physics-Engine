#include <iostream>
#include "application.h"

extern Application* getApplication();

Application* application;

int main(int argc, char* argv[])
{
	application = getApplication();

	application->Execute();

	delete application;	

	return 0;
}