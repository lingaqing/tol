#include "problemS10.h"
using namespace std;

/**
 * Problem S10 object constructor.
 *
 * @param[in] args Command line arguments
 */
problemS10::problemS10(arguments& args) : problem(args){
	setLimits();
	InitialCond();
	countG(x);
}

/**
 * Problem S10 Initial Trajectory
 *
 */
void problemS10::InitialCond(){
	//=======================================
	// Purpose: Develop initial trajectory for SNOPT

	int ind_x, ind_y, ind_z, ind_Va, ind_gamma, ind_chi, ind_phi, ind_CL, ind_dphi, ind_dCL, ind_T, ind_dt;
	double t, w_s,xdot, ydot, zdot, xddot, yddot, zddot, xs, ys, zs, Va, chi, phi, gam, CL, phidoti, CLdoti, T;
	double W1, W2, W3, dtraj_a1, dtraj_a2, dtraj_a3, diff_chi, mag_dtraj_a;
	double r1_1, r1_2, r1_3, r2_3, r3_1, r3_2, r3_3;
	//double W[3] = {0.0, 0.0, 0.0};
	//double JW[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double *x_wind = new double[3];
	double pre_chi, m, L, D, pre_phi, pre_CL;
	double an1, an2, an3, mag_an;
	//double *dxi = new double[sn.numstates];
	//double *state = new double[sn.numstates];
	//double *control = new double[(sn.numinp-sn.numstates)];
	int ii;

	//added by will
	double tfinal = 20;
	double dt = tfinal/sn.ts;
	double xAmp = 100;
	double yAmp = 100;
	double zAmp = 0;

	t = 0.0;
	//xt = (P.xg - P.xi)/(P.tfinal);

	w_s = 2.0*M_PI/(tfinal);
	for (ii = 0; ii <= (sn.ts); ii++)
	{
	  //===============
	  // Define input vector indices
	  ind_x      = ii*sn.numinp + 1;
	  ind_y      = ii*sn.numinp + 2;
	  ind_z      = ii*sn.numinp + 3;
	  ind_Va     = ii*sn.numinp + 4;
	  ind_gamma  = ii*sn.numinp + 5;
	  ind_chi    = ii*sn.numinp + 6;
	  ind_phi    = ii*sn.numinp + 7;
	  ind_CL     = ii*sn.numinp + 8;
	  ind_dphi   = ii*sn.numinp + 9;
	  ind_dCL    = ii*sn.numinp + 10;
	  ind_T      = ii*sn.numinp + 11;
	  ind_dt     = 0;

	//=============
	// Define circular traj
//	if (1)
//	{
//		xs = xAmp*sin(w_s*t) + xi;
//		ys = (-yAmp*cos(w_s*t) + yAmp + yi);
//		zs = zAmp*cos(w_s*t) - zAmp + zi;
//		xdot = w_s*xAmp*cos(w_s*t);
//		ydot = (w_s*yAmp*sin(w_s*t));
//		zdot = -w_s*zAmp*sin(w_s*t);
//		xddot = -w_s*w_s*xAmp*sin(w_s*t);
//		yddot = (w_s*w_s*yAmp*cos(w_s*t));
//		zddot = -w_s*w_s*zAmp*cos(w_s*t);
//	}

	if (1)
	{
		xs = xAmp*sin(w_s*t) - xAmp + xi;
		ys = (-yAmp*cos(w_s*t) + yi);
		zs = zAmp*cos(w_s*t) - zAmp + zi;
		xdot = w_s*xAmp*cos(w_s*t);
		ydot = (w_s*yAmp*sin(w_s*t));
		zdot = -w_s*zAmp*sin(w_s*t);
		xddot = -w_s*w_s*xAmp*sin(w_s*t);
		yddot = (w_s*w_s*yAmp*cos(w_s*t));
		zddot = -w_s*w_s*zAmp*cos(w_s*t);
	}


	//==============
	// Compute wind terms
	x_wind[0] = xs; x_wind[1] = ys; x_wind[2] = zs;
	//WindModel(x_wind, ii*P.dt,W, JW); Shouldn't be needed since initial trajectory never leaves z=0. -Will
	W1 = 0;//W[0];
	W2 = 0;//W[1];
	W3 = 0;//W[2];

	//==============
	// Air relative velocity
	dtraj_a1 = xdot - W1;
	dtraj_a2 = ydot - W2;
	dtraj_a3 = zdot - W3;
	mag_dtraj_a = sqrt(dtraj_a1*dtraj_a1 + dtraj_a2*dtraj_a2 + dtraj_a3*dtraj_a3);

	//==============
	// Va, gamma, chi
	Va = mag_dtraj_a;
	chi = atan2(dtraj_a2,dtraj_a1);
	gam = atan2(-dtraj_a3,sqrt(dtraj_a1*dtraj_a1 + dtraj_a2*dtraj_a2));

	if (ii > 0)
	{
		diff_chi = chi - pre_chi;
		while (((diff_chi) < -M_PI)||((diff_chi) > M_PI))
		{
			if (diff_chi < -M_PI)
			{
				m = ceil((-M_PI - diff_chi)/(2.0*M_PI));
				chi = chi + 2.0*M_PI*m;
			}
			if (diff_chi > M_PI)
			{
				m = floor((M_PI - diff_chi)/(2.0*M_PI));
				chi = chi + 2.0*M_PI*m;
			}
			diff_chi = chi - pre_chi;
		}
	}

	//==============
	// Compute r1
	r1_1 = dtraj_a1/mag_dtraj_a;
	r1_2 = dtraj_a2/mag_dtraj_a;
	r1_3 = dtraj_a3/mag_dtraj_a;

	//==============
	// Compute an
	an1 = -xddot*(r1_1*r1_1 - 1.0) - r1_1*r1_2*yddot - r1_1*r1_3*(zddot - g);
	an2 = -yddot*(r1_2*r1_2 - 1.0) - r1_1*r1_2*xddot - r1_2*r1_3*(zddot - g);
	an3 = -(zddot - g)*(r1_3*r1_3 - 1.0) - r1_1*r1_3*xddot - r1_2*r1_3*yddot;
	mag_an = sqrt(an1*an1 + an2*an2 + an3*an3);

	//==============
	// Compute r3,r2
	r3_1 = -an1/mag_an;
	r3_2 = -an2/mag_an;
	r3_3 = -an3/mag_an;
	//r2_1 = r1_3*r3_2 - r3_3*r1_2;
	//r2_2 = -r1_3*r3_1 + r3_3*r1_1;
	r2_3 = r3_1*r1_2 - r3_2*r1_1;

	//=============
	// Compute phi, CL, T
	phi = atan2(r2_3,r3_3);
	L = ac.mm*mag_an;
	CL = 2.0*L/(rho*Va*Va*ac.SS);
	D = 0.5*rho*Va*Va*ac.SS*(ac.Cd0 + CL*CL/(M_PI*ac.AR*ac.ee));
	T = ac.mm*(r1_1*xddot + r1_2*yddot + r1_3*(zddot-g)) + D;

	if (ii == 0)
	{
		phidoti = 0.0;
		CLdoti  = 0.0;
	}
	else
	{
		phidoti = (phi - pre_phi)/dt;
		CLdoti  = (CL - pre_CL)/dt;
	}
	pre_phi = phi;
	pre_CL  = CL;

	//===============
	// Store trajectory
	x[ind_x]        = xs;
	x[ind_y]        = ys;
	x[ind_z]        = zs;
	x[ind_Va]       = Va;
	x[ind_gamma]    = gam;
	x[ind_chi]      = chi;
	x[ind_phi]      = phi;
	x[ind_CL]       = CL;
	x[ind_dphi]     = phidoti;
	x[ind_dCL]      = CLdoti;
	x[ind_T]        = T;
	x[ind_dt]       = dt;

	//===============
	// Fix initial states
//	if (ii == 0)
//	{
//		//turned in class variables to replace P. struct
//		Vai = Va;
//		gammai = gam;
//		chii = chi;
//		phii = phi;
//		CLi = CL;
//	}

	pre_chi = chi;
	t = t + dt;
	}

	//==========
	// Populate phidot,CLdot at t=0
	x[9]    = x[ind_dphi];
	x[10]   = x[ind_dCL];

	FILE* ifile;
	ifile = fopen("Ioutput.txt","w");
	for(int i = 0; i < n; i++){
		fprintf(ifile, "%.6f\n", x[i]);
	}
	fclose(ifile);
}

/**
 * Problem S10 Cost Function
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 */
void problemS10::cost(double x[], double F[])
{
	double xs,ys,T,dt,r, R, dR;
	int ii;
	double sump, int_p, sumT, int_T;
	sump = 0.0;
	R = rg;
	sumT = 0.0;
	for (ii = 0; ii <= (sn.ts); ii++)
	{
	    //===============
	    // Define input vector indices
	    xs        = x[ii*sn.numinp + 1];
	    ys        = x[ii*sn.numinp + 2];
	    T         = x[ii*sn.numinp + 11];
	    dt        = x[0];

	    //===============
	    // Cost
	    r     = sqrt( (xs - xg)*(xs-xg) + (ys-yg)*(ys-yg) );
	    dR    = (r - R)*(r-R);
	    int_p = dR;
	    int_T = T*T;

	    //===============
	    // Cost Summation
//	    if (r <= R)
//	    {
//	        int_p = 0.0;
//	    }
	    sump = sump + int_p;
	    sumT = sumT + int_T;

	}

	//=====================
	// Compute cost function
	F[0] = 0.5*gn.kT*sumT + 0.5*gn.kp*sump + gn.kdt*dt;
}

/**
 * Problem S10 Boundary Constraints
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 */
void problemS10::boundaryConstraints(double x[], double F[])
{
    //=====================
    // Include boundary constraints
	int ind_Fend;
	double chi_m, delz;
    int Pkills = 0; //added by Will
    if (sn.numbounds > 0)
    {
        if (Pkills == 1)
        {
            chi_m = 2.0*M_PI;//P.chi_d?
            delz = 0.0;
        }
        else
        {
            chi_m = 2.0*M_PI;
            delz = 0.0;
        }
        ind_Fend    = (neF) - sn.numbounds;
        F[ind_Fend] = x[sn.ts*sn.numinp+1]     - x[1]; ind_Fend = ind_Fend + 1; // x(tf) - x(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+2]     - x[2]; ind_Fend = ind_Fend + 1; // y(tf) - y(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+3]     - x[3] - delz; ind_Fend = ind_Fend + 1; // z(tf) - z(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+4]    - x[4]; ind_Fend = ind_Fend + 1; // Va(tf) - Va(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+5] 	- x[5]; ind_Fend = ind_Fend + 1; // gam(tf) - gam(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+6]   - x[6] - chi_m; ind_Fend = ind_Fend + 1; // chi(tf) - chi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+7]   - x[7]; ind_Fend = ind_Fend + 1; // phi(tf) - phi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+8]    - x[8]; ind_Fend = ind_Fend + 1; // CL(tf) - CL(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+9]  - x[9]; ind_Fend = ind_Fend + 1; // dphi(tf) - dphi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+10]   - x[10]; ind_Fend = ind_Fend + 1; // dCL(tf) - dCL(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+11]     - x[11]; ind_Fend = ind_Fend + 1; // T(tf) - T(t0) = 0
    }
}

/**
 * Problem S10 Cost Gradient
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 * @param[out] Gs Computed value for the Jacobian
 */
double problemS10::costGradient(double x[], int Fnum, int xnum, int tf, int tx)
{
	double Gs;
	double r, R;
	int nocost = 0;
	double xs, ys, zs, Va, gam, chi, phi, CL, dphi, dCL, T, dt;
	double tabG[12]= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	//---------------
	// Extract input vector at time tx
	xs    = x[tx*sn.numinp + 1];
	ys    = x[tx*sn.numinp + 2];
	zs    = x[tx*sn.numinp + 3];
	Va    = x[tx*sn.numinp + 4];
	gam   = x[tx*sn.numinp + 5];
	chi   = x[tx*sn.numinp + 6];
	phi   = x[tx*sn.numinp + 7];
	CL    = x[tx*sn.numinp + 8];
	dphi  = x[tx*sn.numinp + 9];
	dCL   = x[tx*sn.numinp + 10];
	T     = x[tx*sn.numinp + 11];
	dt    = x[0];


    //------------
    // Compute r,R
    r   = sqrt( (xs - xg)*(xs-xg) + (ys-yg)*(ys-yg) );
    R   = rg;
//    if (r <= R)
//    {
//        nocost = 1;
//    }

    //------
    // Position integration -> sum(p)
    if (xnum == 0)
    {
        Gs = gn.kp*(r-R)*(xs - xg)/r;
        Gnonzero = 1;
        if (nocost == 1)
        {
            Gs = 0.0;
        }
    }
    if (xnum == 1)
    {
        Gs = gn.kp*(r-R)*(ys - yg)/r;
        Gnonzero = 1;
        if (nocost == 1)
        {
            Gs = 0.0;
        }
    }

    //----
    // Thrust component
    if (xnum == 10)
    {
        Gs = gn.kT*T;
        Gnonzero = 1;
    }

    //----
    // Time component
    if (xnum == 11)
    {
        Gs = gn.kdt;
        Gnonzero = 1;
    }

    return Gs;

}

/**
 * Problem S10 Boundary Gradient
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 * @param[out] Gs Computed value for the Jacobian
 */
double problemS10::boundaryGradients(double x[], int Fnum, int xnum, int tf, int tx)
{
	double Gs;
	//-----------------
	// Boundary constraints
	// only use:.....x............................y............................z............................Va.............................gamma..........................chi............................phi............................CL...............................dphi...........................dCL..............................T
	if ( ((xnum == 0)&&(Fnum == 9))||((xnum == 1)&&(Fnum == 10)) ||((xnum == 2)&&(Fnum == 11)) || ((xnum == 3)&&(Fnum == 12)) || ((xnum == 4)&&(Fnum == 13)) || ((xnum == 5)&&(Fnum == 14)) || ((xnum == 6)&&(Fnum == 15)) || ((xnum == 7)&&(Fnum == 16)) || ((xnum == 8)&&(Fnum == 17)) || ((xnum == 9)&&(Fnum == 18)) || ((xnum == 10)&&(Fnum == 19)) )
	{
		if (tx == 0)
		{
			Gs = -1.0;
			Gnonzero = 1;
		}
		if ((tx == sn.ts))
		{
			Gs = 1.0;
			Gnonzero = 1;
		}
	}
	return Gs;
}
