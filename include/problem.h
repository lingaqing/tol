#ifndef PROBLEM_H_
#define PROBLEM_H_

#include "parameters.h"
#include "arguments.h"
#include "mongo/client/dbclient.h"
#include <chrono>
#include "snopt/snoptProblem.hpp"
#include "snopt/snopt.h"
#include "DefineFG.h"
#include "json/json.h"
#include <fstream>
#include <stdexcept>
#include <cstdio>

class problem
{
public:
	//You must have a valid reason to expose the method/member
	virtual ~problem();							//REASON: Delete called by tol.cpp after execution
	void runSNOPT();							//REASON: Called by tol.cpp to run SNOPT after being setup
	void modelWind(double x[]);					//REASON: Called by DefineFG.cpp (SNOPT)
	void computeF(double x[], double F[]);		//REASON: Called by DefineFG.cpp (SNOPT)
	void computeG(double x[], double G[]);		//REASON: Called by DefineFG.cpp (SNOPT)
	void writeJSON(std::string filename);		//REASON: Called by tol.cpp to write a JSON file
	void writeTXT(std::string filename);		//REASON: Called by tol.cpp to write a TXT file
	bool debug;									//REASON: Flag set in tol.cpp to enable or disable debug outputs
	void countG2(double x[]);


protected:
	//CLASS METHODS
	problem(arguments& args);
	void setLimits();
	void cacheWind();
	void dynamicConstraints(double x[], double F[]);
    double dynamicsGradients(double x[], int Fnum, int xnum, int tf, int tx);
	void countG(double x[]);


	//virtual methods for children
	virtual void InitialCond()=0;
	//virtual void TabulatedG(double& Gs, int Fnum, int xnum, int tf, int tx, double x[], int& Gnonzero)=0;
	virtual void cost(double x[], double F[])=0;
	virtual void boundaryConstraints(double x[], double F[])=0;
	virtual double costGradient(double x[], int Fnum, int xnum, int tf, int tx)=0;
	virtual double boundaryGradients(double x[], int Fnum, int xnum, int tf, int tx)=0;

	//CLASS MEMBERS

	//OBJECTS
	aircraft ac;
	gain gn;
	limit lm;
	snopt sn;
	mongo::DBClientConnection winddb;

	//STRUCTS
	struct winddoc
	{
	   public:
	      winddoc(double x, double y, double z, double u, double v,double w) : x(x), y(y), z(z), u(u), v(v), w(w){}
	      double x;
	      double y;
	      double z;
	      double u;
	      double v;
	      double w;
	};

	//CONSTANTS
	const double g = 9.81;
	const double rho = 1.2682;
	const double pi = M_PI;

	//MISC
	bool stitch_previous = false;
	int Pwindmodel;
	double east, north, up;
	double xg,yg,zg;
	double rg;
	std::string mission;
	std::string aircraft_type;
	std::string rootpath;
	int cachenx, cacheny, cachenz;
	int cache_east, cache_north, cache_up;
	double xspacing = 150;
	double yspacing = 150;
	double zspacing = 150;
	std::vector<std::vector<std::vector<winddoc>>> cache;
	double xi,yi,zi,Vai,gammai, chii, phii, CLi, Va1, Va2, gamma1, gamma2, chi1, chi2, phi1, phi2, CL1, CL2, phidot1, phidot2,CLdot1,CLdot2;
	double EastFromDatum, NorthFromDatum, UpFromDatum;
	std::vector<double> xwind;
	std::vector<double> ywind;
	std::vector<double> zwind;
	std::vector<double> uwind;
	std::vector<double> vwind;
	std::vector<double> wwind;
	double Wx = 0, Wy = 0, Wz = 0;
	double dWx_dx = 0, dWx_dy = 0, dWx_dz = 0;
	double dWy_dx = 0, dWy_dy = 0, dWy_dz = 0;
	double dWz_dx = 0, dWz_dy = 0, dWz_dz = 0;
	std::vector<double> u,v,w,du_dx,du_dy,du_dz,dv_dx,dv_dy,dv_dz,dw_dx,dw_dy,dw_dz;
	std::vector<double> F_sparse, x_sparse, tf_sparse, tx_sparse;
	double Vai_low, gami_low, chii_low, phii_low, CLi_low, dphii_low, dCLi_low, Ti_low;
	double Vai_upp, gami_upp, chii_upp, phii_upp, CLi_upp, dphii_upp, dCLi_upp, Ti_upp;
	int Gnonzero;

	//SNOPT STUFF
	int n;
	int neA;
	int neF;
	int lenA;
	int *iAfun;
	int *jAvar;
	double *A;
	int neG;
	int lenG;
	int *iGfun;
	int *jGvar;
	double *x;
	double *xlow;
	double *xupp;
	double *xmul;
	int   *xstate;
	double *F;
	double *Flow;
	double *Fupp;
	double *Fmul;
	int   *Fstate;
	int nxnames;
	int nFnames;
	char *xnames;
	char *Fnames;
	int   ObjRow;
	double ObjAdd;

	//parallel speed test
	//double *Galt;
};

#endif
