#ifndef PROBLEMG7_H_
#define PROBLEMG7_H_

#include "problem.h"

class problemG7 : public problem
{
public:
	/**@brief problem LT constructor*/
	problemG7(arguments& args);

protected:
	void InitialCond();
	//void TabulatedG(double& Gs, int Fnum, int xnum, int tf, int tx, double x[], int& Gnonzero);
	void cost(double F[], double x[]);
	void boundaryConstraints(double F[], double x[]);

	double costGradient(double x[], int Fnum, int xnum, int tf, int tx);
	double boundaryGradients(double x[], int Fnum, int xnum, int tf, int tx);

	void RotateYaw(double []);

	double chi_d;
};

#endif
