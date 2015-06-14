#ifndef ARGUMENTS_H_
#define ARGUMENTS_H_

#include "Python.h"
#include <string>
#include <iostream>

class arguments {

public:
	arguments(PyObject* args);
	arguments(char** args);
	double east,north,up;
	double east_goal,north_goal,up_goal,radius_goal;
	char *char_ac;
	char *char_mission;
	double xi,yi,zi,Vai,gami,chii,phii,CLi,dphii,dCLi,Ti;
	std::string root_path;
	std::string aircraft,mission;
	bool stitch_previous;
};

#endif /* ARGUMENTS_H_ */
