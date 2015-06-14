#include "tol.h"
using namespace std;
problem *prob = NULL;

int mission_select(arguments& args){
	bool exec_SNOPT = true;

	//Create problem object
	if (args.mission == "G7")
	{
		cout << "TOL STATUS: Detected request for mission G7." << endl;
		prob = new problemG7(args);	//Loitering Thrust problem
	}
	else if (args.mission == "S10")
	{
		cout << "TOL STATUS: Detected request for mission S10." << endl;
		prob = new problemS10(args);	//Loitering NO Thrust problem
	}
	else
	{
		cout << "TOL STATUS: Mission code \"" << args.mission << "\" not recognized.";
		exec_SNOPT = false;
	}

	//execute SNOPT for selected problem
	if (exec_SNOPT == true){
		prob->runSNOPT();
		cout << "TOL STATUS: Run Complete!" << endl;
		//prob->writeTXT("snopt_results.txt");
		prob->writeJSON("snopt_results.json");
	}
	cout << "TOL STATUS: Cleaning up...";
	delete prob;
	cout << "success" << endl;
	return 0;
}

int main( int argc, char *argv[])
{
	if (argc < 10)
	{
		cout << "SNOPT requires exactly 9 arguments to run." << endl;
		cout << "Only " << argc << " were given." << endl;
	}
	else
	{
		remove( "snopt_log.out" );
		remove( "GeneralWind.out" );
		arguments args(argv);		//parse C++ arguments into py_args object
		mission_select(args);	//Plan mission selected by user
	}
	return 0;
}

/*
 * Expose "execute_SNOPT" to python
 */
static PyObject* execute_SNOPT( PyObject* self, PyObject* args)
{
	arguments arg(args);		//parse python arguments into C++ py_args object
	mission_select(arg);	//Execute mission selected by python
	return Py_BuildValue("i",0);
}

/*
 * Bind Python function names to our C functions
 */
static PyMethodDef libtol_methods[] = {{"execute_SNOPT", execute_SNOPT, METH_VARARGS},{NULL, NULL}};

/*
 * Python calls this to let us initialize our module
 */
PyMODINIT_FUNC initlibtol()
{
  (void) Py_InitModule("libtol", libtol_methods);
}
