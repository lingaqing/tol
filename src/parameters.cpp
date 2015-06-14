#include "parameters.h"
using namespace std;

/**
 * Parameters constructor (unused)
 */
parameters::parameters(){}

/**
 * Reads parameters from .param files
 *
 * @param[in] filepath file path to be read
 */
void parameters::readparams(string filepath){
	read_params.clear();
	ifstream input(filepath);
	string line;
	char delim = '//';
	for( string line; getline( input, line ); )
	{
	    stringstream ss(line);
	    string item;
	    getline(ss, item, delim);
	    try
	    {
	    	read_params.push_back(stod(item));
	    }
	    catch(exception& e)
		{
	    	cout << "TOL STATUS: Reading file: " << line << "...";
		}
	}
	input.close();
}

/**
 * aircraft parameter constructor
 *
 * @param[in] aircraftname
 * @param[in] root_path relative path to the project root
 */
aircraft::aircraft(std::string aircraftname, std::string root_path){
	string filepath =  root_path + "aircraft/" + aircraftname + ".param";
	this->readparams(filepath);
	bool size_ok = (read_params.size() == 15) ? true : false;
	if (size_ok)
	{
		mm = read_params[0];
		b = read_params[1];
		SS = read_params[2];
		ee = read_params[3];
		AR = read_params[4];
		Cd0 = read_params[5];
		CLmin = read_params[6];
		CLmax = read_params[7];
		phimax = read_params[8]*M_PI/180.0;
		Vamin = read_params[9];
		Vamax = read_params[10];
		gammamax = read_params[11]*M_PI/180.0;
		phidotmax = read_params[12]*M_PI/180.0;
		Tmin = read_params[13];
		Tmax = read_params[14];
	}
	else
	{
		throw std::length_error( "Wrong number of parameters for "+aircraftname+".param" );
	}
	cout << "success" << endl;
}

/**
 * gain parameter constructor
 *
 * @param[in] problemtype Mission type
 * @param[in] root_path relative path to the project root
 */
gain::gain(std::string problemtype, std::string root_path){
	string filepath =  root_path + "problems/" + problemtype + "/gains.param";
	this->readparams(filepath);
	bool size_ok = (read_params.size() == 5) ? true : false;
	if (size_ok)
	{
		kT= read_params[0];
		kp = read_params[1];
		kv = read_params[2];
		ka = read_params[3];
		kdt = read_params[4];
	}
	else
	{
		throw std::length_error( "Wrong number of parameters for "+problemtype+".param" );
	}
	cout << "success" << endl;
}

/**
 * limit parameter constructor
 *
 * @param[in] problemtype Mission type
 * @param[in] root_path relative path to the project root
 */
limit::limit(std::string problemtype, std::string root_path){
	string filepath =  root_path + "/problems/" + problemtype + "/limits.param";
	this->readparams(filepath);
	bool size_ok = (read_params.size() == 8) ? true : false;
	if (size_ok)
	{
		dtmin = read_params[0];
		dtmax = read_params[1];
		xmin = read_params[2];
		xmax = read_params[3];
		ymin = read_params[4];
		ymax = read_params[5];
		zmin = read_params[6];
		zmax = read_params[7];
	}
	else
	{
		throw std::length_error( "Wrong number of parameters for "+problemtype+".param" );
	}
	cout << "success" << endl;
}

/**
 * snopt parameter constructor
 *
 * @param[in] problemtype Mission type
 * @param[in] root_path relative path to the project root
 */
snopt::snopt(std::string problemtype, std::string root_path){
	string filepath =  root_path + "/problems/" + problemtype + "/snopt.param";
	this->readparams(filepath);
	bool size_ok = (read_params.size() == 6) ? true : false;
	if (size_ok)
	{
		ts = read_params[0];
		numinp = read_params[1];
		numstates = read_params[2];
		numbounds = read_params[3];
		opt_tol = read_params[4];
		feas_tol = read_params[5];
	}
	else
	{
		throw std::length_error( "Wrong number of parameters for "+problemtype+".param" );
	}
	cout << "success" << endl;
}
