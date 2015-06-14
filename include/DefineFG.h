#ifndef DEFINEFG_H_
#define DEFINEFG_H_
#include "snopt/snoptProblem.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void DEFINEGusrfg_( int   *Status, int *n,    double x[],
		 	 	 	int    *needF,  int *neF,  double F[],
					int    *needG,  int *neG,  double G[],
					char       *cu,     int *lencu,
					int    iu[],    int *leniu,
					double ru[],    int *lenru );
#ifdef __cplusplus
}
#endif
#endif
