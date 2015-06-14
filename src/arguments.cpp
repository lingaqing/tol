#include "arguments.h"
using namespace std;

/**
 * Arguments constructor for a Python call
 *
 * @param[in] args Arguments from the python function call
 */
arguments::arguments(PyObject* args){
	stitch_previous = false;
	if (!PyArg_ParseTuple(args, "(ddddddddddd)(dddd)ss",
								&east,&north,&up,
								&Vai,&gami,&chii,
								&phii,&CLi,&dphii,
								&dCLi,&Ti,
								&east_goal,&north_goal,&up_goal,&radius_goal,
								&char_ac,&char_mission)){}

	if (Vai != 0){
		stitch_previous = true;
	}
	aircraft.assign(char_ac);
	mission.assign(char_mission);
	root_path = "../";
}

/**
 * Arguments constructor for a command line call
 *
 * @param[in] args The command line arguments
 */
arguments::arguments(char** args){ //used if run program from command line (not using python)
	stitch_previous = false;
	char_ac = {};
	char_mission = {};
	east = atof(args[1]);
	north  = atof(args[2]);
	up = atof(args[3]);
	east_goal = atof(args[4]);
	north_goal = atof(args[5]);
	up_goal = atof(args[6]);
	radius_goal = atof(args[7]);
	aircraft = std::string(args[8]);
	mission = std::string(args[9]);
	root_path = "./";
}


