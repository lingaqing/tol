#ifndef PROBLEMS10_H_
#define PROBLEMS10_H_

#include "problem.h"

class problemS10 : public problem
{
public:
	problemS10(arguments& args);

protected:
	void InitialCond();
	//void TabulatedG(double& Gs, int Fnum, int xnum, int tf, int tx, double x[], int& Gnonzero);

	void cost(double F[], double x[]);
	void boundaryConstraints(double F[], double x[]);

	double costGradient(double x[], int Fnum, int xnum, int tf, int tx);
	double boundaryGradients(double x[], int Fnum, int xnum, int tf, int tx);

	void RotateYaw(double v[]);
};

#endif
