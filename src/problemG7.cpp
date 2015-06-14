#include "problemG7.h"
using namespace std;

/**
 * Problem G7 object constructor.
 *
 * @param[in] args Command line arguments
 */
problemG7::problemG7(arguments& args) : problem(args){
	setLimits();
	InitialCond();
	countG(x);
}

/**
 * Problem G7 Initial Trajectory
 *
 */
void problemG7::InitialCond(){
	cout << "TOL STATUS: Generating Initial Trajectory...";
	//=======================================
	// Purpose: Develop initial trajectory for SNOPT

	int ind_x, ind_y, ind_z, ind_Va, ind_gamma, ind_chi, ind_phi, ind_CL, ind_dphi, ind_dCL, ind_T, ind_dt;
	double t, w_s,xdot, ydot, zdot, xddot, yddot, zddot, xs, ys, zs, Va, gam, chi, phi, CL, phidoti, CLdoti, T;
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
	double tfinal = 10;
	double dt = tfinal/sn.ts;
	double xAmp = 40;
	double yAmp = 0;
	double zAmp = 0;

	double pos[3] = {0.0,0.0,0.0};

	t = 0.0;
	//xt = (P.xg - P.xi)/(P.tfinal);
	//xt = Vai;

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
	// Define sinusoidal
	if (1)
	{
        xs = xAmp/tfinal*t + xi;
        ys = -yAmp*cos(w_s*t) + yAmp + yi;
        zs = zAmp*cos(w_s*t) - zAmp + zi;
        xdot = xAmp/tfinal;
        ydot = yAmp*w_s*sin(w_s*t);
        zdot = -zAmp*w_s*sin(w_s*t);
        xddot = 0.0;
        yddot = yAmp*w_s*w_s*cos(w_s*t);
        zddot = -zAmp*w_s*w_s*cos(w_s*t);
	}

    //==============
    // Rotate to desired direction
    pos[0] = xs; pos[1] = ys; pos[2] = zs;
    RotateYaw(pos);
    xs = pos[0]; ys = pos[1]; zs = pos[2];


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
	chi = atan2(dtraj_a2,dtraj_a1) + chi_d;
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

	FILE* ifile;
	ifile = fopen("Ioutput.txt","w");
	for(int i = 0; i < n; i++){
		fprintf(ifile, "%.6f\n", x[i]);
	}
	fclose(ifile);

//	//==========
//	// Populate phidot,CLdot at t=0
//	x[9]    = x[ind_dphi];
//	x[10]   = x[ind_dCL];
	cout << "done" << endl;
}

/**
 * Problem G7 Cost Function
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 */
void problemG7::cost(double x[], double F[])
{
	double xs,ys,T,dt,delx,dely,dist;
	int ii;
	double costsum, int_T;
	costsum = 0.0;
	for (ii = 0; ii <= (sn.ts); ii++)
	{
		  // Define input vector indices
		  xs        = x[ii*sn.numinp + 1];
		  ys        = x[ii*sn.numinp + 2];
	      T         = x[ii*sn.numinp + 11];
	      dt        = x[0];

		  // Cost summation
	      int_T = T*T;
	      costsum = costsum + int_T;
	}

	//=====================
	// Compute cost function
	delx = xs - x[1];
	dely = ys - x[2];
	dist = sqrt( delx*delx + dely*dely );
	F[0] = gn.kT*0.5*costsum + gn.kv*sn.ts*dt/dist;
}

/**
 * Problem G7 Boundary Constraints
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 */
void problemG7::boundaryConstraints(double x[], double F[])
{
    //=====================
    // Include boundary constraints
	int ind_Fend;
    double phi_m, dphi_m, xf,x0,yf,y0,dist,dmax;
    if (sn.numbounds > 0)
    {
        //if (P.kills == 1)
        //{
        //    phi_m = 0.0;
        //    dphi_m = 0.0;
        //}
        //else
        //{
            phi_m = x[7];
            dphi_m = x[9];
        //}
        xf = x[sn.ts*sn.numinp + 1];
        x0 = x[1];
        yf = x[sn.ts*sn.numinp + 2];
        y0 = x[2];
        dist = sqrt( (xf-x0)*(xf-x0) + (yf-y0)*(yf-y0) );
        dmax = sqrt( (xg-x0)*(xg-x0) + (yg-y0)*(yg-y0) );
        ind_Fend    = (neF) - sn.numbounds;
        F[ind_Fend] = xf - x0 - dist*cos(chi_d); ind_Fend = ind_Fend + 1;    // x(tf) - x(t0) - d*cos(chi_d) = 0
        F[ind_Fend] = yf - y0 - dist*sin(chi_d); ind_Fend = ind_Fend + 1;    // y(tf) - y(t0) - d*sin(chi_d) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+3]     - x[3]; ind_Fend = ind_Fend + 1;    // z(tf) - z(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+4]    - x[4]; ind_Fend = ind_Fend + 1;    // Va(tf) - Va(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+5] - x[5]; ind_Fend = ind_Fend + 1;    // gam(tf) - gam(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+6]   - x[6]; ind_Fend = ind_Fend + 1;    // chi(tf) - chi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+7]   - phi_m; ind_Fend = ind_Fend + 1;    // phi(tf) - phi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+8]    - x[8]; ind_Fend = ind_Fend + 1;    // CL(tf) - CL(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+9]  - dphi_m; ind_Fend = ind_Fend + 1;    // dphi(tf) - dphi(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+10]   - x[10]; ind_Fend = ind_Fend + 1;   // dCL(tf) - dCL(t0) = 0
        F[ind_Fend] = x[sn.ts*sn.numinp+11]     - x[11]; ind_Fend = ind_Fend + 1;   // T(tf) - T(t0) = 0
        F[ind_Fend] = dist - dmax;                  ind_Fend = ind_Fend + 1;    // dist <= dmax
    }
}

/**
 * Problem G7 Cost Gradient
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 * @param[out] Gs Computed value for the Jacobian
 */
double problemG7::costGradient(double x[], int Fnum, int xnum, int tf, int tx)
{
	double Gs;
	double r, R;
	int nocost;
	double xs, ys, zs, Va, gam, chi, phi, CL, dphi, dCL, T, dt;
	double tabG[12]= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	double dist,xf,x0,yf,y0;


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

	//--------------
	// Compute trajectory distance
	xf = x[sn.ts*sn.numinp + 1];
	x0 = x[1];
	yf = x[sn.ts*sn.numinp + 2];
	y0 = x[2];
	dist = sqrt( (xf-x0)*(xf-x0) + (yf-y0)*(yf-y0) );


	 //------
	// Position x,y
	//delx = x[P.ts*P.numinp + 1] - x[1];
	if ((xnum == 0)&&(tx == 0))
	{
		Gs = gn.kp*sn.ts*dt*(xf-x0)/(dist*dist*dist);
		Gnonzero = 1;
	}

	if ((xnum == 0)&&(tx == sn.ts))
	{
		Gs = -gn.kp*sn.ts*dt*(xf-x0)/(dist*dist*dist);
		Gnonzero = 1;
	}
	if ((xnum == 1)&&(tx == 0))
	{
		Gs = gn.kp*sn.ts*dt*(yf-y0)/(dist*dist*dist);
		Gnonzero = 1;
	}

	if ((xnum == 1)&&(tx == sn.ts))
	{
		Gs = -gn.kp*sn.ts*dt*(yf-y0)/(dist*dist*dist);
		Gnonzero = 1;
	}

	//------
	// Thrust integration -> sum(T)
	if (xnum == 10)
	{
		Gs = gn.kT*T;
		Gnonzero = 1;
	}

	//------
	// Time
	if (xnum == 11)
	{
		Gs = gn.kp*sn.ts/(dist);
		Gnonzero = 1;
	}

    return Gs;

}

/**
 * Problem G7 Boundary Gradient
 *
 * @param[in] F Constraint vector
 * @param[in] x State vector
 * @param[out] Gs Computed value for the Jacobian
 */
double problemG7::boundaryGradients(double x[], int Fnum, int xnum, int tf, int tx)
{
	double dist,xf,x0,yf,y0;
	//--------------
	// Compute trajectory distance
	xf = x[sn.ts*sn.numinp + 1];
	x0 = x[1];
	yf = x[sn.ts*sn.numinp + 2];
	y0 = x[2];
	dist = sqrt( (xf-x0)*(xf-x0) + (yf-y0)*(yf-y0) );

	double Gs = 0;
    // only use:......z............................Va.............................gamma..........................chi............................phi............................CL.............................dphi...........................dCL..............................T
    if ( ((xnum == 2)&&(Fnum == 11)) ||((xnum == 3)&&(Fnum == 12)) || ((xnum == 4)&&(Fnum == 13)) || ((xnum == 5)&&(Fnum == 14)) || ((xnum == 6)&&(Fnum == 15)) || ((xnum == 7)&&(Fnum == 16)) || ((xnum == 8)&&(Fnum == 17)) || ((xnum == 9)&&(Fnum == 18)) || ((xnum == 10)&&(Fnum == 19)) )
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

    //------
    // Directional constraint: F9,x
    if ( ( xnum == 0 )&&( Fnum == 9) )
    {
        if (tx == 0)
        {
            Gs = -1.0 + ((xf-x0)/dist)*cos(chi_d);
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = 1.0 - ((xf-x0)/dist)*cos(chi_d);
            Gnonzero = 1;
        }
    }
    //------
    // Directional constraint: F9,y
    if ( ( xnum == 1 )&&( Fnum == 9) )
    {
        if (tx == 0)
        {
            Gs = ((yf-y0)/dist)*cos(chi_d);
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = -((yf-y0)/dist)*cos(chi_d);
            Gnonzero = 1;
        }
    }
    //------
    // Directional constraint: F10,x
    if ( ( xnum == 0 )&&( Fnum == 10) )
    {
        if (tx == 0)
        {
            Gs = ((xf-x0)/dist)*sin(chi_d);
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = -((xf-x0)/dist)*sin(chi_d);
            Gnonzero = 1;
        }
    }
    //------
    // Directional constraint: F10,y
    if ( ( xnum == 1 )&&( Fnum == 10) )
    {
        if (tx == 0)
        {
            Gs = -1.0 + ((yf-y0)/dist)*sin(chi_d);
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = 1.0 - ((yf-y0)/dist)*sin(chi_d);
            Gnonzero = 1;
        }
    }

    //-----
    // Max distance constraint,x
    if ( ( xnum == 0)&&( Fnum == 20) )
    {
        if (tx == 0)
        {
            Gs = -(xf-x0)/dist;
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = (xf-x0)/dist;
            Gnonzero = 1;
        }
    }
    //-----
    // Max distance constraint,y
    if ( ( xnum == 1)&&( Fnum == 20) )
    {
        if (tx == 0)
        {
            Gs = -(yf-y0)/dist;
            Gnonzero = 1;
        }
        if (tx == sn.ts)
        {
            Gs = (yf-y0)/dist;
            Gnonzero = 1;
        }
    }
	return Gs;
}

/**
 * Problem G7 RotateYaw
 *
 * @param[in] v to be rotated
 */
void problemG7::RotateYaw(double v[])
{
	double M11,M12,M13,M21,M22,M23,M31,M32,M33;
	double v_p[3] = {0.0, 0.0, 0.0};
	chi_d = atan2(yg-yi,xg-xi);     // Define desired inertial course angle to follow

	//-------------
	// Setup rotation matrix
	M11 = cos(chi_d);    M12 = -sin(chi_d);    M13 = 0.0;
	M21 = sin(chi_d);    M22 = cos(chi_d);     M23 = 0.0;
	M31 = 0.0;           M32 = 0.0;            M33 = 1.0;

	//-------------
	// Rotate vector
	v_p[0] = M11*v[0] + M12*v[1] + M13*v[2];
	v_p[1] = M21*v[0] + M22*v[1] + M23*v[2];
	v_p[2] = M31*v[0] + M32*v[1] + M33*v[2];

	//------------
	// Output rotated components
	v[0] = v_p[0]; v[1] = v_p[1]; v[2] = v_p[2];

}
