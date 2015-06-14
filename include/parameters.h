#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <sstream>
#include <stdexcept>

class parameters{
public:
	parameters();
	void readparams(std::string filepath);
protected:
	double params_array[62];
	std::vector<double> read_params;
};

class aircraft : parameters{
public:
	aircraft(std::string aircraftname, std::string root_path);
	double mm;
	double b;
	double SS;
	double ee;
	double AR;
	double Cd0;
	double CLmin;
	double CLmax;
	double phimax;
	double Vamin;
	double Vamax;
	double gammamax;
	double phidotmax;
	double Tmin;
	double Tmax;
};

class gain : parameters{
public:
	gain(std::string problemtype, std::string root_path); //problem specific constructor
	double kT;
	double kp;
	double kv;
	double ka;
	double kdt;
};

class limit : parameters{
public:
	limit(std::string problemtype, std::string root_path);
	double dtmin;
	double dtmax;
	double xmax;
	double ymax;
	double zmax;
	double xmin;
	double ymin;
	double zmin;
};

class snopt : parameters{
public:
	snopt(std::string problemtype, std::string root_path);
	int ts;
	int numinp;
	int numstates;
	int numbounds;
	double opt_tol;
	double feas_tol;
};

#endif
