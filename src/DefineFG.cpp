#include "DefineFG.h"
#include "global_objects.h"
#include <ctime>
using namespace std;
/**
 * User-defined function for SNOPT to call during execution
 *
 */
void DEFINEGusrfg_( int    *Status, int *n,    double x[],
	       int   *needF,  int *neF,  double F[],
	       int    *needG,  int *neG,  double G[],
	       char       *cu,     int *lencu,
	       int   iu[],    int *leniu,
	       double ru[],    int *lenru )
{
	FILE* xfile;
	xfile = fopen("Xoutput.txt","w");
	for(int i = 0; i < *n; i++){
		fprintf(xfile, "%.14f\n", x[i]);
	}
	fclose(xfile);

	//Compute wind for guessed trajectory
	prob->modelWind(x);

  if ( *needF > 0 ) {
	prob->computeF(x, F);

	FILE* ffile;
	ffile = fopen("Foutput.txt","w");
	for(int i = 0; i < *neF; i++){
		fprintf(ffile, "%.14f\n", F[i]);
	}
	fclose(ffile);
  }
  if ( *needG > 0 ) {
    prob->computeG(x, G);

    //prob->countG2(x);

    FILE* gfile;
	gfile = fopen("Goutput.txt","w");
	for(int i = 0; i < *neG; i++){
		fprintf(gfile, "%.14f\n", G[i]);
	}
	fclose(gfile);
  }
}
