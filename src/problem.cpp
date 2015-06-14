#include "problem.h"
#include <omp.h>
using namespace std;

/**
 * Problem object constructor.
 * Only called by a mission constructor.
 * Initializes the four parameter objects
 *
 * @param[in] problemtype Defines the mission-specific problem type.
 * @param[in] args Command line arguments
 */
problem::problem(arguments& args): ac(args.aircraft,args.root_path),gn(args.mission,args.root_path),lm(args.mission,args.root_path),sn(args.mission,args.root_path)
{
	cout << "TOL STATUS: Building " << args.mission << "..." << endl;
	debug = true;

	//Unpack arguments from python/command line
	east = args.east;
	north = args.north;
	up = args.up;

	//CONVERTING GOALS FROM ENU TO NED!!!
	yg = args.east_goal;
	xg = args.north_goal;
	zg = -args.up_goal;
	rg = args.radius_goal;

	aircraft_type = args.aircraft;
	mission = args.mission;

	//Stitch initial trajectory states based on previous trajectory (if one exists)
	if (args.stitch_previous)
	{
		double Va_slack = 2;
		double gam_slack = 10*pi/180.0;
		double phi_slack = 10*pi/180.0;
		double CL_slack = 0.2;
		double T_slack = 5;

		Vai_low = max(args.Vai - Va_slack,ac.Vamin); 		Vai_upp = min(args.Vai + Va_slack,ac.Vamax);
		gami_low = max(args.gami - gam_slack,-ac.gammamax); gami_upp = min(args.gami + gam_slack,ac.gammamax);
		chii_low = args.chii; 								chii_upp = args.chii;
		phii_low = max(args.phii - phi_slack,-ac.phimax); 	phii_upp = min(args.phii + phi_slack,ac.phimax);
		CLi_low = max(args.CLi - CL_slack,ac.CLmin); 		CLi_upp = min(args.CLi + CL_slack,ac.CLmax);
		dphii_low = args.dphii; 							dphii_upp = args.dphii;
		dCLi_low = args.dCLi; 								dCLi_upp = args.dCLi;
		Ti_low = max(args.Ti - T_slack,ac.Tmin); 			Ti_upp = min(args.Ti + T_slack, ac.Tmax);
	}
	else
	{
		Vai_low = ac.Vamin; 		Vai_upp = ac.Vamax;
		gami_low = -ac.gammamax; 	gami_upp = ac.gammamax;
		chii_low = -pi; 			chii_upp = pi;
		phii_low = -ac.phimax; 		phii_upp = ac.phimax;
		CLi_low = ac.CLmin; 		CLi_upp = ac.CLmax;
		dphii_low = -ac.phidotmax; 	dphii_upp = ac.phidotmax;
		dCLi_low = -ac.phidotmax; 	dCLi_upp = ac.phidotmax;
		Ti_low = ac.Tmin; 			Ti_upp = ac.Tmax;
	}

	// Connect to Wind Field Database
	try{
		cout << "TOL STATUS: Connecting to Wind Field Database...";
		mongo::client::initialize();
		winddb.connect("focalhost:27017");
		Pwindmodel = 3;
		cout << "successful" << endl;
		cout << "TOL STATUS: Caching wind for current aircraft location...";
		cacheWind();
		cout << "successful" << endl;
	}
	catch (exception& e){
		//cout << "ERROR: " << e.what() << endl;
		cout << "failure" << endl;
		cout << "TOL STATUS: Using artificial linear boundary layer instead" << endl;
		Pwindmodel = 1;
	}

	if (args.mission == "S10")
	{
	//Loiter S10
	 xi = 0;
	 yi = 0;
	 zi = 0;
	 Vai = 8;
	 gammai = 0;
	 chii = 0;
	 phii = 0;
	 CLi = 0.9;

	 Va1 = 4;
	 Va2 = 50;
	 gamma1 = 0;
	 gamma2 = 0;
	 chi1 = -1.7453292519943296e+18;
	 chi2 = 1.7453292519943296e+18;
	 phi1 = -1.5707963267948966;
	 phi2 = 1.5707963267948966;
	 CL1 = -0.5;
	 CL2 = 3;
	 phidot1 = -3.4906585039886591;
	 phidot2 = 3.4906585039886591;
	 CLdot1 = -200;
	 CLdot2 = 200;
	}

	if (args.mission == "G7")
	{
	 //Guidance G7
	 xi = 0;
	 yi = 0;
	 zi = 0;
	 Vai = 8;
	 gammai = 0;
	 chii = 0;
	 phii = 0;
	 CLi = 0.9;

	 Va1 = 4;
	 Va2 = 50;
	 gamma1 = 0.0*M_PI/180.0;
	 gamma2 = 0.0*M_PI/180.0;
	 chi1 = -1e20*M_PI/180.0;
	 chi2 = 1e20*M_PI/180.0;
	 phi1 = -90.0*M_PI/180.0;
	 phi2 = 90.0*M_PI/180.0;
	 CL1 = -0.5;
	 CL2 = 3.0;
	 phidot1 = -3.4906585039886591;
	 phidot2 = 3.4906585039886591;
	 CLdot1 = -200.0;
	 CLdot2 = 200.0;
	}

	// Allocate wind variables
	u.resize(sn.ts+1);
	v.resize(sn.ts+1);
	w.resize(sn.ts+1);
	du_dx.resize(sn.ts+1);
	du_dy.resize(sn.ts+1);
	du_dz.resize(sn.ts+1);
	dv_dx.resize(sn.ts+1);
	dv_dy.resize(sn.ts+1);
	dv_dz.resize(sn.ts+1);
	dw_dx.resize(sn.ts+1);
	dw_dy.resize(sn.ts+1);
	dw_dz.resize(sn.ts+1);

	// Allocate SNOPT variables
	n          = sn.numinp*(sn.ts+1)+1;						// number of variables in input vector, x
	neF        = (sn.numstates)*(sn.ts)+1+sn.numbounds;		// number of components of vector F, which holds objective function(+1), nonlinear constraint functions, and boundary constraint(+P.sn.numbounds)
	lenA       = neF*n;										// dimension of array A which contains the linear constraints

	iAfun     = new int[lenA];								// holds row component of A
	jAvar     = new int[lenA];								// holds column component of A
	A      = new double[lenA];								// array containing the unwrapped linear constraint matrix A

	lenG       = neF*n;                   					// dimension of G which holds the gradient of the nonlinear constraints
	iGfun     = new int[lenG];								// holds row component of G
	jGvar     = new int[lenG];								// holds column component of G

	x      = new double[n];									// decision vector
	xlow   = new double[n];									// lower bound on input
	xupp   = new double[n];									// upper bound on input
	xmul   = new double[n];									// vector of dual variables for bound constraints
	xstate = new    int[n];									// initial values for input vector

	F      = new double[neF];								// nonlinear constraints
	Flow   = new double[neF];								// lower bound on constraints
	Fupp   = new double[neF];								// upper bound on constraints
	Fmul   = new double[neF];								// estimate of lagrangian multipliers for constraints??
	Fstate = new int[neF];									// initial values for constraints F

	nxnames    = 1;											// Gives number of variables in xname
	nFnames    = 1;											// Gives number of constraint names in Fname
	xnames       = new char[nxnames*8];						// Holds 8-character name for variables
	Fnames       = new char[nFnames*8];						// Holds 8-character name for constraint equations

	ObjRow  = 0;											// Indicates where the objective is defined in F
	ObjAdd  = 0;											// Used for printing purposes, is constant added to objective row. Typically this is zero.
	neA = 0;
	neG = 0;

	//Allocate Jacobian indices
	F_sparse.resize(lenG);
	x_sparse.resize(lenG);
	tf_sparse.resize(lenG);
	tx_sparse.resize(lenG);


}

/**
 * Sets SNOPT state (x) and constraint (F) limits.
 *
 */
void problem::setLimits()
{
	cout << "TOL STATUS: Setting state and constraint limits...";

	  //====================================
	  // Define indices
	  int ind_x, ind_y, ind_z, ind_Va, ind_gamma, ind_chi, ind_phi, ind_CL, ind_dphi, ind_dCL, ind_T, ind_dt;
	  int ii, ind_Fx, ind_Fy, ind_Fz, ind_FVa, ind_Fgamma, ind_Fchi, ind_Fphi, ind_FCL, ind_Fend;

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

	  	  //===============
	  	  // Define F vector indices
	  	  ind_Fx     = ii*(sn.numstates) + 1;
	  	  ind_Fy     = ii*(sn.numstates) + 2;
	  	  ind_Fz     = ii*(sn.numstates) + 3;
	  	  ind_FVa    = ii*(sn.numstates) + 4;
	  	  ind_Fgamma = ii*(sn.numstates) + 5;
	  	  ind_Fchi   = ii*(sn.numstates) + 6;
	      ind_Fphi   = ii*(sn.numstates) + 7;
	      ind_FCL    = ii*(sn.numstates) + 8;


	  	  //===============
	  	  // Define initial condition bounds
//	  	  if (ii == 0)
//	  	  {
//	  		xlow[ind_x]     =         xi;  						xupp[ind_x]     =        xi;		// Bounds on x
//	  		xlow[ind_y]     =         yi;  						xupp[ind_y]     =        yi;		// Bounds on y
//	  		xlow[ind_z]     =         zi;  						xupp[ind_z]     =        zi;		// Bounds on z
//	  		xlow[ind_Va]    = 	   Vai_low;  					xupp[ind_Va]    =      Vai_upp;     // Bounds on Va
//	  		xlow[ind_gamma] =     0;						xupp[ind_gamma] =     0;     			// Bounds on gamma
//	  		xlow[ind_chi]   =     chii_low;  					xupp[ind_chi]   =     chii_upp;     // Bounds on chi
//	  		xlow[ind_phi]   =     phii_low; 					xupp[ind_phi]   =     phii_upp;     // Bounds on phi
//	  		xlow[ind_CL]    =      CLi_low;  					xupp[ind_CL]    =      CLi_upp;     // Bounds on CL
//	        xlow[ind_dphi]  =    dphii_low;						xupp[ind_dphi]  =    dphii_upp;     // Bounds on phidot
//	        xlow[ind_dCL]   =     dCLi_low;						xupp[ind_dCL]   =     dCLi_upp;     // Bounds on CLdot
//	        xlow[ind_T]     =       Ti_low;						xupp[ind_T]     =       Ti_upp;		// Bounds on T
//	        xlow[ind_dt]    =     lm.dtmin;  					xupp[ind_dt]    =     lm.dtmax;     // Bounds on dt
//	  	  }

	  	  if (ii == 0)
	  	  {
	  		xlow[ind_x]     =         xi;  						xupp[ind_x]     =        xi;		// Bounds on x
	  		xlow[ind_y]     =         yi;  						xupp[ind_y]     =        yi;		// Bounds on y
	  		xlow[ind_z]     =         zi;  						xupp[ind_z]     =        zi;		// Bounds on z
	  		xlow[ind_Va]    = 	   Va1;  					xupp[ind_Va]    =      Va2;     // Bounds on Va
	  		xlow[ind_gamma] =      gamma1;						xupp[ind_gamma] =     gamma2;     // Bounds on gamma
	  		xlow[ind_chi]   =     chi1;  					xupp[ind_chi]   =     chi2;     // Bounds on chi
	  		xlow[ind_phi]   =     phi1; 					xupp[ind_phi]   =     phi2;     // Bounds on phi
	  		xlow[ind_CL]    =      CL1;  					xupp[ind_CL]    =      CL2;     // Bounds on CL
	        xlow[ind_dphi]  =    phidot1;						xupp[ind_dphi]  =    phidot2;     // Bounds on phidot
	        xlow[ind_dCL]   =     CLdot1;						xupp[ind_dCL]   =     CLdot2;     // Bounds on CLdot
	        xlow[ind_T]     =       0;						xupp[ind_T]     =       1e20;		// Bounds on T
	        xlow[ind_dt]    =     lm.dtmin;  					xupp[ind_dt]    =     lm.dtmax;     // Bounds on dt
	  	  }

	  	  //===============
	  	  // Define bounds on the rest of the trajectory
	  	  if (ii > 0)
	  	  {
	  		xlow[ind_x]     =       lm.xmin;  xupp[ind_x]     =        lm.xmax;	// Bounds on x
	  		xlow[ind_y]     =       lm.ymin;  xupp[ind_y]     =        lm.ymax;	// Bounds on y
	  		xlow[ind_z]     =       lm.zmin;  xupp[ind_z]     =        lm.zmax;	// Bounds on z
	  		xlow[ind_Va]    =      ac.Vamin;  xupp[ind_Va]    =       ac.Vamax;	// Bounds on Va
	          xlow[ind_gamma] =  -ac.gammamax;  xupp[ind_gamma] =    ac.gammamax;	// Bounds on gamma
	  		xlow[ind_chi]   =        -1e20;  xupp[ind_chi]   =          1e20;	// Bounds on chi
	  		xlow[ind_phi]   =    -ac.phimax;  xupp[ind_phi]   =      ac.phimax;	// Bounds on phi
	  		xlow[ind_CL]    =      ac.CLmin;  xupp[ind_CL]    =       ac.CLmax;	// Bounds on CL
	          xlow[ind_dphi]  = -ac.phidotmax;  xupp[ind_dphi]  =   ac.phidotmax;   // Bounds on phidot
	          xlow[ind_dCL]   = -ac.phidotmax;  xupp[ind_dCL]   =   ac.phidotmax;   // Bounds on CLdot
	          xlow[ind_T]     =       ac.Tmin;  xupp[ind_T]     =        ac.Tmax;	// Bounds on T
	  	  }


	  	  //====================================
	  	  // Set the state components
	        xstate[ind_x]   =  0;  xstate[ind_y]     =  0;  xstate[ind_z]      =  0;
	        xstate[ind_Va]  =  0;  xstate[ind_gamma] =  0;  xstate[ind_chi]    =  0;
	        xstate[ind_phi] =  0;  xstate[ind_CL]    =  0;  xstate[ind_dphi]   =  0;
	        xstate[ind_dCL] =  0;  xstate[ind_T]     =  0;  xstate[ind_dt]     =  0;

	  	  //====================================
	  	  // Setup F vector- cost function
	  	  if (ii == 0)
	  	  {
	  		  Fmul[0] = 0;
	  		  Flow[0] = -1e20; Fupp[0] = 1e20;
	  	  }

	  	  //====================================
	  	  // Setup F vector- constraints
	  	  if (ii <= (sn.ts-1))
	  	  {
	  		Flow[ind_Fx]      =   0.0;   Fupp[ind_Fx]      =   0.0;		// Bounds on x-xdot*dt-xi = 0
	  		Flow[ind_Fy]      =   0.0;   Fupp[ind_Fy]      =   0.0;		// Bounds on y-ydot*dt-yi = 0
	  		Flow[ind_Fz]      =   0.0;   Fupp[ind_Fz]      =   0.0;		// Bounds on z-zdot*dt-zi = 0
	  		Flow[ind_FVa]     =   0.0;   Fupp[ind_FVa]     =   0.0;		// Bounds on Va-Vadot*dt-Vai = 0
	  		Flow[ind_Fgamma]  =   0.0;   Fupp[ind_Fgamma]  =   0.0;		// Bounds on gamma-gammadot*dt-gammai = 0
	  		Flow[ind_Fchi]    =   0.0;   Fupp[ind_Fchi]    =   0.0;		// Bounds on chi-chidot*dt-chii = 0
	        Flow[ind_Fphi]    =   0.0;   Fupp[ind_Fphi]    =   0.0;     // Bounds on phi-phidot*dt-phii = 0
	        Flow[ind_FCL]     =   0.0;   Fupp[ind_FCL]     =   0.0;     // Bounds on CL-CLdot*dt-CLi = 0

	  	    //===================================
	  	    // Setup F vector- Fmul
	  		Fmul[ind_Fx]   = 0.0; Fmul[ind_Fy]     = 0.0; Fmul[ind_Fz] = 0.0;
	  		Fmul[ind_FVa]  = 0.0; Fmul[ind_Fgamma] = 0.0; Fmul[ind_Fchi] = 0.0;
	        Fmul[ind_Fphi] = 0.0; Fmul[ind_FCL]    = 0.0;
	  	  }

	    }
	  //===============
	  // Boundary Constraint
	  if(sn.numbounds > 0)
	  {

		  if (mission == "S10")
			  {
			  ind_Fend = neF - sn.numbounds;
			  for (ii = 0;ii < sn.numbounds;ii++)
			  {
				  Fmul[ind_Fend+ii] = 0.0;
				  Flow[ind_Fend+ii] = 0.0;
				  Fupp[ind_Fend+ii] = 0.0;
			  }
		  }
		  else if (mission == "G7")
		  {
			int kills = 0;
			ind_Fend = neF - sn.numbounds;
			for (ii = 0;ii < sn.numbounds;ii++)
			{
				if ( (ii == (sn.numbounds-1))&&( kills == 0) )
				{
					Fmul[ind_Fend+ii] = 0.0;
					Flow[ind_Fend+ii] = -1e20;   // dist <= dmax if no kill shot
					Fupp[ind_Fend+ii] = 0.0;
				}
				else
				{
					Fmul[ind_Fend+ii] = 0.0;
					Flow[ind_Fend+ii] = 0.0;
					Fupp[ind_Fend+ii] = 0.0;
				}
			}
		  }
		  else
		  {
			  throw std::invalid_argument( "Problem "+mission+" not recognized." );
		  }
	  }
	  cout << "done" << endl;
}

/**
 * Caches a small subset of wind from the wind field database.
 *
 */
void problem::cacheWind(){
		string ns = "straka.t000000_f5700";

		cout << "TOL STATUS: Found " << winddb.count(ns) << " wind data points in " << ns << endl;

		//Datum
		static const double datum[] = { 40.144832, -105.242877, 1681};	//lat,lon,alt (meters MSL)
		vector<double> Datum (datum, datum + sizeof(datum) / sizeof(datum[0]) );

		//Aircraft
		vector<double> Aircraft(3);
		Aircraft[0] =  40.146630; 	//lat
		Aircraft[1] = -105.239674; 	//lon
		Aircraft[2] = 1681+100;	  	//100 meters AGL

		//=====================================
		//Find X,Y,Z between Datum and Aircraft

		//Convert degrees to radians:
		double lat1 = Datum[0] * pi/180;
		double lon1 = Datum[1] * pi/180;

		double lat2 = Aircraft[0] * pi/180;
		double lon2 = Aircraft[1] * pi/180;

		double dlat  = lat2-lat1;
		double dlong = lon2-lon1;

		// Haversine formula:
		double R = 6371000;
		double a = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlong/2.0)*sin(dlong/2.0);
		double c = 2.0 * atan2( sqrt(a), sqrt(1-a) );
		double d = R * c;	//distance
		double b = atan2( sin(dlong)*cos(lat2), cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlong) );

		EastFromDatum = d*cos(pi/2-b);	//x meters offset from Datum (ENU)
		NorthFromDatum = d*sin(pi/2-b);	//y meters offset from Datum (ENU)
		UpFromDatum = Aircraft[2] - Datum[2]; //z meters offset from Datum (ENU), should never be negative lol

		//temporary override to put us in custom storm locations
		EastFromDatum = 17400;	//x meters offset from Datum (ENU)
		NorthFromDatum = 25800;	//y meters offset from Datum (ENU)
		UpFromDatum = 200; //z meters offset from Datum (ENU), should never be negative lol

		//Count number of entries in x,y,z:
		mongo::BSONObj cache_east_query =  winddb.distinct(ns, "x", MONGO_QUERY(
				"x" << mongo::GTE << EastFromDatum - 5*yspacing << mongo::LTE << EastFromDatum + 5*yspacing <<
				"y" << mongo::GTE << NorthFromDatum - 5*xspacing << mongo::LTE << NorthFromDatum + 5*xspacing <<
				"z" << mongo::GTE << UpFromDatum - 2*zspacing << mongo::LTE << UpFromDatum + 2*zspacing));

		mongo::BSONObj cache_north_query =  winddb.distinct(ns, "y", MONGO_QUERY(
				"x" << mongo::GTE << EastFromDatum - 5*yspacing << mongo::LTE << EastFromDatum + 5*yspacing <<
				"y" << mongo::GTE << NorthFromDatum - 5*xspacing << mongo::LTE << NorthFromDatum + 5*xspacing <<
				"z" << mongo::GTE << UpFromDatum - 2*zspacing << mongo::LTE << UpFromDatum + 2*zspacing));

		mongo::BSONObj cache_up_query =  winddb.distinct(ns, "z", MONGO_QUERY(
				"x" << mongo::GTE << EastFromDatum - 5*yspacing << mongo::LTE << EastFromDatum + 5*yspacing <<
				"y" << mongo::GTE << NorthFromDatum - 5*xspacing << mongo::LTE << NorthFromDatum + 5*xspacing <<
				"z" << mongo::GTE << UpFromDatum - 2*zspacing << mongo::LTE << UpFromDatum + 2*zspacing));

		//Get the data

		std::auto_ptr<mongo::DBClientCursor> cursor = winddb.query(ns,MONGO_QUERY(
				"x" << mongo::GTE << EastFromDatum - 5*yspacing << mongo::LTE << EastFromDatum + 5*yspacing <<
				"y" << mongo::GTE << NorthFromDatum - 5*xspacing << mongo::LTE << NorthFromDatum + 5*xspacing <<
				"z" << mongo::GTE << UpFromDatum - 2*zspacing << mongo::LTE << UpFromDatum + 2*zspacing),400);

		//store the data in a 3D vector of winddocs.
		cache_east = cache_east_query.nFields();
		cache_north = cache_north_query.nFields();
		cache_up = cache_up_query.nFields();

		for(int i = 0; i < cache_east; i++){
			vector < vector < winddoc > > w;
		    cache.push_back( w );
		    for(int j = 0; j < cache_north; j++){
		    	vector <winddoc> v;
		    	cache[i].push_back( v );
		    	for(int k = 0; k < cache_up; k++){
		    		mongo::BSONObj p = cursor->next();
		    		cache[i][j].push_back( winddoc(   p.getField("x").numberDouble(),
		    									      p.getField("y").numberDouble(),
		    									      p.getField("z").numberDouble(),
													  (p.getField("u").numberDouble() == -32768 ? 0 : p.getField("u").numberDouble()),	//check if value is -32768 (e.g. NaN)
													  (p.getField("v").numberDouble() == -32768 ? 0 : p.getField("v").numberDouble()),
													  (p.getField("w").numberDouble() == -32768 ? 0 : p.getField("w").numberDouble())));
		    	}
		    }
		 }
	}

/**
 * Defines the wind model.
 * The following models are currently available:
 * 0. No wind
 * 1. Linear Boundary Layer
 * 2. Vertical Thermal
 * 3. Wind Field Database (MongoDB)
 * 4. Thermals
 * 5. Cyclic Wind
 *
 * @param[in] problemtype Defines the mission-specific problem type.
 * @param[in] args Command line arguments
 */
void problem::modelWind(double x[])
{
	switch (Pwindmodel)
	{
	    // No wind
	    case 0:
	    {
	    	for (int i = 0; i <= sn.ts; i++)
	    	{
	    		u[i] = 0;
	    		v[i] = 0;
	    		w[i] = 0;
				du_dx[i] = 0;
				du_dy[i] = 0;
				du_dz[i] = 0;
				dv_dx[i] = 0;
				dv_dy[i] = 0;
				dv_dz[i] = 0;
				dw_dx[i] = 0;
				dw_dy[i] = 0;
				dw_dz[i] = 0;
	    	}
	        break;
	    }

	    // Linear boundary layer (Dynamic soaring)
	    case 1:
	    {
	    	double zs;
	    	double Vref = 2.4;
	    	double href = 10;
	    	for (int i = 0; i <= sn.ts; i++)
	    	{
	    		u[i] = 0;
	    		v[i] = 0;
	    		w[i] = 0;
				du_dx[i] = 0;
				du_dy[i] = 0;
				du_dz[i] = 0;
				dv_dx[i] = 0;
				dv_dy[i] = 0;
				dv_dz[i] = 0;
				dw_dx[i] = 0;
				dw_dy[i] = 0;
				dw_dz[i] = 0;

				//ENU <- NED
				zs = -x[i*sn.numinp + 3];
				v[i] = -Vref*zs/href;
				dv_dz[i] = -Vref/href;

	    	}



	        break;
	    }

	    // Thermal soaring
	    case 2:
	    {
//	        r_ths = (xs-P.xth)*(xs-P.xth) + (ys-P.yth)*(ys-P.yth);
//	        wz = -P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift));
//	        dWz_dx = 2.0*P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift))*(xs-P.xth)/(P.rlift*P.rlift);
//	        dWz_dy = 2.0*P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift))*(ys-P.yth)/(P.rlift*P.rlift);
	        break;
	    }

	    // Storm data
	    case 3:
	    {
	    	double xs,ys,zs;	int xi, yi, zi;
	    	double xrel,yrel,zrel,dx,dy,dz,zeta,eta,mu;
	    	for (int ii = 0; ii <= sn.ts; ii++)
	    	{
	    		//ENU <- NED
				xs = x[ii*sn.numinp + 2] + EastFromDatum;
				ys = x[ii*sn.numinp + 1] + NorthFromDatum;
				zs = -x[ii*sn.numinp + 3] + UpFromDatum;

				//find cube in windcache that surrounds point
				for (xi = 0; xi < cache_north; xi++){
					if ((xs - cache[xi][0][0].x) < xspacing){
						break;
					}
				}

				for (yi = 0; yi < cache_east; yi++){
					if ((ys - cache[0][yi][0].y) < yspacing){
						break;
					}
				}

				for (zi = 0; zi < cache_up; zi++){
					if ((zs - cache[0][0][zi].z) < zspacing){
						break;
					}
				}

				double u_corner[8];
				u_corner[0] = cache[xi][yi][zi].u;
				u_corner[1] = cache[xi+1][yi][zi].u;
				u_corner[2] = cache[xi][yi+1][zi].u;
				u_corner[3] = cache[xi+1][yi+1][zi].u;
				u_corner[4] = cache[xi][yi][zi+1].u;
				u_corner[5] = cache[xi+1][yi][zi+1].u;
				u_corner[6] = cache[xi][yi+1][zi+1].u;
				u_corner[7] = cache[xi+1][yi+1][zi+1].u;

				double v_corner[8];
				v_corner[0] = cache[xi][yi][zi].v;
				v_corner[1] = cache[xi+1][yi][zi].v;
				v_corner[2] = cache[xi][yi+1][zi].v;
				v_corner[3] = cache[xi+1][yi+1][zi].v;
				v_corner[4] = cache[xi][yi][zi+1].v;
				v_corner[5] = cache[xi+1][yi][zi+1].v;
				v_corner[6] = cache[xi][yi+1][zi+1].v;
				v_corner[7] = cache[xi+1][yi+1][zi+1].v;

				double w_corner[8];
				w_corner[0] = cache[xi][yi][zi].w;
				w_corner[1] = cache[xi+1][yi][zi].w;
				w_corner[2] = cache[xi][yi+1][zi].w;
				w_corner[3] = cache[xi+1][yi+1][zi].w;
				w_corner[4] = cache[xi][yi][zi+1].w;
				w_corner[5] = cache[xi+1][yi][zi+1].w;
				w_corner[6] = cache[xi][yi+1][zi+1].w;
				w_corner[7] = cache[xi+1][yi+1][zi+1].w;


				//normalized positioning coordinates within cube of interest
				xrel = (xs - cache[xi][0][0].x);
				yrel = (ys - cache[0][yi][0].y);
				zrel = (zs - cache[0][0][zi].z);
				dx = xspacing;
				dy = yspacing;
				dz = zspacing;
				zeta = xrel/dx;
				eta = yrel/dy;
				mu = zrel/dz;

				//Define shape functions (Eight Node Trilinear Hexahedron)
				double N[8];
				N[0] = (1-zeta)*(1-eta)*(1-mu); 	//N1
				N[1] = zeta*(1-eta)*(1-mu);	//N2
				N[2] = (1-zeta)*eta*(1-mu);	//N3
				N[3] = zeta*eta*(1-mu);	//N4
				N[4] = (1-zeta)*(1-eta)*mu;	//N5
				N[5] = zeta*(1-eta)*mu;	//N6
				N[6] = (1-zeta)*eta*mu;	//N7
				N[7] = zeta*eta*mu;	//N8

				//Calculate interpolated u,v,w
				u[ii] = 0;
				v[ii] = 0;
				w[ii] = 0;
				for (int i = 0; i < 8; i++){
					//u[ii] += N[i]*u_corner[i];//interpolated_u
					v[ii] += N[i]*v_corner[i];//interpolated_v
					//w[ii] += N[i]*w_corner[i];//interpolated_w
				}

				//Now, on to getting wind gradients
				double NwrtX[8];
				double NwrtY[8];
				double NwrtZ[8];

				//Shape function derivatives with respect to x
				NwrtX[0] = -((yrel/dy - 1.0)*(zrel/dz - 1.0))/dx;  //N1 wrt xrel
				NwrtX[1] = ((yrel/dy - 1.0)*(zrel/dz - 1.0))/dx;   //N2 wrt xrel
				NwrtX[2] = (yrel*(zrel/dz - 1.0))/(dx*dy);  //N3 wrt xrel
				NwrtX[3] = -(yrel*(zrel/dz - 1.0))/(dx*dy);   //N4 wrt xrel
				NwrtX[4] = (zrel*(yrel/dy - 1.0))/(dx*dz);   //N5 wrt xrel
				NwrtX[5] = -(zrel*(yrel/dy - 1.0))/(dx*dz);  //N6 wrt xrel
				NwrtX[6] = -(yrel*zrel)/(dx*dy*dz);   //N7 wrt xrel
				NwrtX[7] = (yrel*zrel)/(dx*dy*dz);  //N8 wrt xrel

				//Shape function derivatives with respect to y
				NwrtY[0] = -((xrel/dx - 1.0)*(zrel/dz - 1.0))/dy;  //N1 wrt yrel
				NwrtY[1] = (xrel*(zrel/dz - 1.0))/(dx*dy);   //N2 wrt yrel
				NwrtY[2] = ((xrel/dx - 1.0)*(zrel/dz - 1.0))/dy;  //N3 wrt yrel
				NwrtY[3] = -(xrel*(zrel/dz - 1.0))/(dx*dy);   //N4 wrt yrel
				NwrtY[4] = (zrel*(xrel/dx - 1.0))/(dy*dz);   //N5 wrt yrel
				NwrtY[5] = -(xrel*zrel)/(dx*dy*dz);  //N6 wrt yrel
				NwrtY[6] = -(zrel*(xrel/dx - 1.0))/(dy*dz);   //N7 wrt yrel
				NwrtY[7] = (xrel*zrel)/(dx*dy*dz);  //N8 wrt yrel

				//Shape function derivatives with respect to z
				NwrtZ[0] = -((xrel/dx - 1.0)*(yrel/dy - 1.0))/dz;  //N1 wrt zrel
				NwrtZ[1] = (xrel*(yrel/dy - 1.0))/(dx*dz);   //N2 wrt zrel
				NwrtZ[2] = (yrel*(xrel/dx - 1.0))/(dy*dz);  //N3 wrt zrel
				NwrtZ[3] = -(xrel*yrel)/(dx*dy*dz);   //N4 wrt zrel
				NwrtZ[4] = ((xrel/dx - 1.0)*(yrel/dy - 1.0))/dz;   //N5 wrt zrel
				NwrtZ[5] = -(xrel*(yrel/dy - 1.0))/(dx*dz);  //N6 wrt zrel
				NwrtZ[6] = -(yrel*(xrel/dx - 1.0))/(dy*dz);   //N7 wrt zrel
				NwrtZ[7] = (xrel*yrel)/(dx*dy*dz);  //N8 wrt zrel

				//Calculate wind gradients
				du_dx[ii] = 0;
				du_dy[ii] = 0;
				du_dz[ii] = 0;
				dv_dx[ii] = 0;
				dv_dy[ii] = 0;
				dv_dz[ii] = 0;
				dw_dx[ii] = 0;
				dw_dy[ii] = 0;
				dw_dz[ii] = 0;
				for (int i = 0; i < 8; i++){
					//du_dx[ii] += NwrtX[i]*u_corner[i];//interpolated_du_dx
					//du_dy[ii] += NwrtY[i]*u_corner[i];//interpolated_du_dy
					//du_dz[ii] += NwrtZ[i]*u_corner[i];//interpolated_du_dz
					dv_dx[ii] += NwrtX[i]*v_corner[i];//interpolated_dv_dx
					dv_dy[ii] += NwrtY[i]*v_corner[i];//interpolated_dv_dy
					dv_dz[ii] += NwrtZ[i]*v_corner[i];//interpolated_dv_dz
					//dw_dx[ii] += NwrtX[i]*w_corner[i];//interpolated_dw_dx
					//dw_dy[ii] += NwrtY[i]*w_corner[i];//interpolated_dw_dy
					//dw_dz[ii] += NwrtZ[i]*w_corner[i];//interpolated_dw_dz
				}
	    	}
	        break;
	    }

	    // Thermals (2) (source and sink)
	    case 4:
	    {
//	        r_ths = (xs-P.xth)*(xs-P.xth) + (ys-P.yth)*(ys-P.yth);
//	        r_ths2 = (xs-P.xth2)*(xs-P.xth2) + (ys-P.yth2)*(ys-P.yth2);
//	        wz = -P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift)) + -P.Vcore2*exp(-(r_ths2)/(P.rlift2*P.rlift2));
//	        dWz_dx = 2.0*P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift))*(xs-P.xth)/(P.rlift*P.rlift) + 2.0*P.Vcore2*exp(-(r_ths2)/(P.rlift2*P.rlift2))*(xs-P.xth2)/(P.rlift2*P.rlift2);
//	        dWz_dy = 2.0*P.Vcore*exp(-(r_ths)/(P.rlift*P.rlift))*(ys-P.yth)/(P.rlift*P.rlift) + 2.0*P.Vcore2*exp(-(r_ths2)/(P.rlift2*P.rlift2))*(ys-P.yth2)/(P.rlift2*P.rlift2);
	        break;
	    }

	    // Cyclic wind
	    case 5:
	    {
//	        dx = xs - P.xth;
//	        dy = ys - P.yth;
//	        r = sqrt(dx*dx + dy*dy);
//	        wx = -P.Vcore*dy/r;
//	        wy = P.Vcore*dx/r;
//	        dWx_dx = P.Vcore*dx*dy/(r*r*r);
//	        dWx_dy = P.Vcore*dy*dy/(r*r*r) - P.Vcore/r;
//	        dWy_dx = -P.Vcore*dx*dx/(r*r*r) + P.Vcore/r;
//	        dWy_dy = -P.Vcore*dx*dy/(r*r*r);
//	        if ((dx == 0)&&(dy == 0))
//	        {
//	            wx = 0.0;
//	            wy = 0.0;
//	            dWx_dx = 0.0;
//	            dWx_dy = 0.0;
//	            dWy_dx = 0.0;
//	            dWy_dy = 0.0;
//	        }
	        break;
	    }

	    default:
	    {
	        break;
	    }
	}



		FILE* wfile;
		wfile = fopen("Woutput.txt","w");
		for(int i = 0; i < sn.ts+1; i++){
			fprintf(wfile, "%.6f ", u[i]);
			fprintf(wfile, "%.6f ", v[i]);
			fprintf(wfile, "%.6f ", w[i]);
			fprintf(wfile, "%.6f ", du_dx[i]);
			fprintf(wfile, "%.6f ", du_dy[i]);
			fprintf(wfile, "%.6f ", du_dz[i]);
			fprintf(wfile, "%.6f ", dv_dx[i]);
			fprintf(wfile, "%.6f ", dv_dy[i]);
			fprintf(wfile, "%.6f ", dv_dz[i]);
			fprintf(wfile, "%.6f ", dw_dx[i]);
			fprintf(wfile, "%.6f ", dw_dy[i]);
			fprintf(wfile, "%.6f\n", dw_dz[i]);
		}
		fclose(wfile);
}

/**
 * Computes Constraints vector
 *
 * @param[in] x state vector
 * @param[in] F Constraint vector
 */
void problem::computeF(double x[], double F[]){
	// Compute cost function
	cost(x,F);

	// Compute dynamic constraints
	dynamicConstraints(x,F);

	// Compute boundary constraints
	boundaryConstraints(x,F);
}

/**
 * Computes Jacobian vector
 *
 * @param[in] x state vector
 * @param[in] G Jacobian vector
 */
void problem::computeG(double x[], double G[]){
	auto start = std::chrono::system_clock::now();
    //#pragma omp parallel for
	for (int Gind = 0; Gind < neG; Gind++)
	{
		if (F_sparse[Gind] == 0)
		{
			//Compute cost gradients
			G[Gind] = costGradient(x, F_sparse[Gind], x_sparse[Gind], tf_sparse[Gind], tx_sparse[Gind]);
		}
		else if(F_sparse[Gind] <= sn.numstates)
		{
			//Compute dynamics gradients
			G[Gind] = dynamicsGradients(x, F_sparse[Gind], x_sparse[Gind], tf_sparse[Gind], tx_sparse[Gind]);
		}
		else
		{
			//Compute boundary gradients
			G[Gind] = boundaryGradients(x, F_sparse[Gind], x_sparse[Gind], tf_sparse[Gind], tx_sparse[Gind]);
		}
	}
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//cout << elapsed.count() << endl;
}

/**
 * Counts number of non-zero entries in Jacobian
 *
 * @param[in] x state vector
 */
void problem::countG(double x[])
{
	int Fnum,xnum,tf,tx,pF,px,Gind,reserve_dt;
	double Gs;
	pF = sn.numstates;
	px = sn.numinp;
	Gs = 0.0;
	Gind = 0;
	for (int ii = 0; ii < (neF); ii++)
	{
		for (int jj = 0; jj < (n); jj++)
		{
			//--------------
			// Determine F at time index tf
			Fnum = ii%pF;
			if ((Fnum == 0) && (ii != 0))
			{
				Fnum = pF;
			}
			tf = (ii-1)/pF;
			if (ii >= (neF - sn.numbounds) )
			{
				Fnum = sn.numstates + sn.numbounds - (neF-1-ii);   // F constraint for chi(tf)-chi(t0)
			}

			//--------------
			// Determine x at time index tx
			if (jj == 0)
			{
				xnum = px;
				reserve_dt = Gind;
			}
			else
			{
				xnum = (jj-1)%px;
			}
			tx = (jj-1)/px;

			//--------------
			// Compute G
			if (Fnum == 0)
			{
				//Compute cost gradients
				costGradient(x, Fnum, xnum, tf, tx);
			}
			else if(Fnum <= sn.numstates)
			{
				//Compute dynamics gradients
				dynamicsGradients(x, Fnum, xnum, tf, tx);
			}
			else
			{
				//Compute boundary gradients
				boundaryGradients(x, Fnum, xnum, tf, tx);
			}
			if ( (Gnonzero == 1) || (xnum == px) )
			{
				iGfun[neG] = ii;
				jGvar[neG] = jj;
				F_sparse[neG] = Fnum;
				x_sparse[neG] = xnum;
				tf_sparse[neG] = tf;
				tx_sparse[neG] = tx;
				neG = neG + 1;
				Gind++;

			}

			//--------------
			// Account for dt
			if ( (xnum == (Fnum-1))&&(tf == tx)&&(Fnum > 0)&&(Fnum <= pF) )
			{
				xnum = px;
				Gs = 0.0;
				Gnonzero = 0;
				//--------------
				// Compute G
				if (Fnum == 0)
				{
					//Compute cost gradients
					costGradient(x, Fnum, xnum, tf, tx);
				}
				else if(Fnum <= sn.numstates)
				{
					//Compute dynamics gradients
					dynamicsGradients(x, Fnum, xnum, tf, tx);
				}
				else
				{
					//Compute boundary gradients
					boundaryGradients(x, Fnum, xnum, tf, tx);
				}
				F_sparse[reserve_dt] = Fnum;
				x_sparse[reserve_dt] = xnum;
				tf_sparse[reserve_dt] = tf;
				tx_sparse[reserve_dt] = tx;
				reserve_dt = -1;
			}

			//--------------
			// Zero out variables
			Gnonzero = 0;
			Gs = 0.0;
		}
	}
	cout << "Done Counting" << endl;
}

/**
 * Computes aircraft dynamics constraints.
 * Uses 3D point mass aircraft model.
 *
 * @param[in] x state vector
 * @param[in] F constraint vector
 *
 */
void problem::dynamicConstraints(double x[], double F[])
{
	int ind_dt, ind_x, ind_y, ind_z, ind_Va, ind_gamma, ind_chi, ind_phi, ind_CL, ind_dphi, ind_dCL, ind_T;
	int ind_Fx, ind_Fy, ind_Fz, ind_FVa, ind_Fgamma, ind_Fchi, ind_Fphi, ind_FCL;
	double Va,gam,chi,phi,CL,T;
	double dx[sn.numstates];

	for (int ii = 0; ii < sn.ts; ii++)
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

		//===============
		// Define current state
		//xs = x[ind_x];
		//ys = x[ind_y];
		//zs = x[ind_z];
		Va = x[ind_Va];
		gam = x[ind_gamma];
		chi = x[ind_chi];
		phi = x[ind_phi];
		CL = x[ind_CL];
		//dphi = x[ind_dphi];
		//dCL = x[ind_dCL];
		T = x[ind_T];
		//dt = x[ind_dt];

		//===============
		// Get Wind at current state (NED <- ENU)
		Wx = v[ii];
		Wy = u[ii];
		Wz = -w[ii];
		dWx_dx = dv_dy[ii];
		dWx_dy = dv_dx[ii];
		dWx_dz = -dv_dz[ii];
		dWy_dx = du_dy[ii];
		dWy_dy = du_dx[ii];
		dWy_dz = -du_dz[ii];
		dWz_dx = -dw_dy[ii];
		dWz_dy = -dw_dx[ii];
		dWz_dz = dw_dz[ii];



//		double Vref = 1;
//		double href = 10;
//        Wy = Vref*x[ii*sn.numinp + 3]/href;
//        dWy_dz = Vref/href;

		//===============
		// Define F vector indices
		ind_Fx     = ii*(sn.numstates) + 1;
		ind_Fy     = ii*(sn.numstates) + 2;
		ind_Fz     = ii*(sn.numstates) + 3;
		ind_FVa    = ii*(sn.numstates) + 4;
		ind_Fgamma = ii*(sn.numstates) + 5;
		ind_Fchi   = ii*(sn.numstates) + 6;
		ind_Fphi   = ii*(sn.numstates) + 7;
		ind_FCL    = ii*(sn.numstates) + 8;

		//===============
		// Compute Derivatives
		dx[0] 	 	= Wx + Va*cos(chi)*cos(gam);
		dx[1] 	 	= Wy + Va*cos(gam)*sin(chi);
		dx[2] 	 	= Wz - Va*sin(gam);
		dx[3] 	 	= T/ac.mm - (Wy + Va*cos(gam)*sin(chi))*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)) - (Wz - Va*sin(gam))*(dWx_dz*cos(chi)*cos(gam) - dWz_dz*sin(gam) + dWy_dz*cos(gam)*sin(chi)) - (Wx + Va*cos(chi)*cos(gam))*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)) - g*sin(gam) - (rho*ac.SS*Va*Va*(ac.Cd0 + CL*CL/(ac.AR*M_PI*ac.ee)))/(2.0*ac.mm);
		dx[4] 	 	= ((Wx + Va*cos(chi)*cos(gam))*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) + (Wz - Va*sin(gam))*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) - g*cos(gam) + (CL*rho*ac.SS*Va*Va*cos(phi))/(2*ac.mm))/Va;
		dx[5] 	 	= -((Wz - Va*sin(gam))*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) + (dWy_dx*cos(chi) - dWx_dx*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) - (CL*rho*ac.SS*Va*Va*sin(phi))/(2.0*ac.mm))/(Va*cos(gam));

		//===============
		// Develop constraints using Forward Euler approx.
		F[ind_Fx]     = x[ind_x+sn.numinp] - dx[0]*x[ind_dt] - x[ind_x];			// x - xdot*t - x1
		F[ind_Fy]     = x[ind_y+sn.numinp] - dx[1]*x[ind_dt] - x[ind_y];			// y - ydot*t - y1
		F[ind_Fz]     = x[ind_z+sn.numinp] - dx[2]*x[ind_dt] - x[ind_z];			// z - zdot*t - z1
		F[ind_FVa]    = x[ind_Va+sn.numinp] - dx[3]*x[ind_dt] - x[ind_Va];			// Va - Vadot*t - Va1
		F[ind_Fgamma] = x[ind_gamma+sn.numinp] - dx[4]*x[ind_dt] - x[ind_gamma];	// gamma - gammadot*t - gamma1
		F[ind_Fchi]   = x[ind_chi+sn.numinp] - dx[5]*x[ind_dt] - x[ind_chi];		// chi - chidot*t - chi1
		F[ind_Fphi]   = x[ind_phi+sn.numinp] - x[ind_dphi]*x[ind_dt] - x[ind_phi]; // phi - phidot*t - phi1
		F[ind_FCL]    = x[ind_CL+sn.numinp] - x[ind_dCL]*x[ind_dt] - x[ind_CL];    // CL - CLdot*t - CL1
	  }
}

/**
 * Computes aircraft dynamics gradients
 * Uses 3D point mass aircraft model.
 *
 * @param[in] x state vector
 * @param[in] Constraint number
 * @param[in] State number
 * @param[in] Which constraint in time
 * @param[in] Which state in time
 * @param[out] Gs Computed value for the Jacobian
 *
 */
double problem::dynamicsGradients(double x[], int Fnum, int xnum, int tf, int tx)
{
	double xs, ys, zs, Va, gam, chi, phi, CL, dphi, dCL, T, dt;
	double tabG[12]= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double r, R;
	int nocost;
	nocost = 0;
	double Gs = 0;

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

	//===============
	// Get Wind at current state (NED <- ENU)
	Wx = v[tx];
	Wy = u[tx];
	Wz = -w[tx];
	dWx_dx = dv_dy[tx];
	dWx_dy = dv_dx[tx];
	dWx_dz = -dv_dz[tx];
	dWy_dx = du_dy[tx];
	dWy_dy = du_dx[tx];
	dWy_dz = -du_dz[tx];
	dWz_dx = -dw_dy[tx];
	dWz_dy = -dw_dx[tx];
	dWz_dz = dw_dz[tx];

	if (tx == tf)
	{
		switch(Fnum)
		{
		//---------------------------------------
		// F1
		case 1:

			//-------
			// Tabulated G
			tabG[0] 	= -1.0;
			tabG[3] 	= -dt*cos(chi)*cos(gam);
			tabG[4] 	= Va*dt*cos(chi)*sin(gam);
			tabG[5] 	= Va*dt*cos(gam)*sin(chi);
			tabG[11]    = - Wx -Va*cos(chi)*cos(gam);

			break;

		//---------------------------------------
		// F2
		case 2:

			//-------
			// Tabulated G
			tabG[1] 	= -1.0;
			tabG[3] 	= -dt*cos(gam)*sin(chi);
			tabG[4] 	= Va*dt*sin(chi)*sin(gam);
			tabG[5] 	= -Va*dt*cos(chi)*cos(gam);
			tabG[11]    = - Wy - Va*cos(gam)*sin(chi);

			break;

		//----------------------------------------
		// F3
		case 3:

			//-------
			// Tabulated G
			tabG[2] 	= -1.0;
			tabG[3] 	= dt*sin(gam);
			tabG[4] 	= Va*dt*cos(gam);
			tabG[11]    = Va*sin(gam) - Wz;

			break;

		//----------------------------------------
		// F4
		case 4:

			//-------
			// Tabulated G
			tabG[3] 	= dt*(cos(chi)*cos(gam)*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)) - sin(gam)*(dWx_dz*cos(chi)*cos(gam) - dWz_dz*sin(gam) + dWy_dz*cos(gam)*sin(chi)) + cos(gam)*sin(chi)*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)) + (rho*ac.SS*Va*(ac.Cd0 + (CL*CL)/(ac.AR*M_PI*ac.ee)))/ac.mm) - 1.0;
			tabG[4] 	= -dt*((Wx + Va*cos(chi)*cos(gam))*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) + (Wz - Va*sin(gam))*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) - g*cos(gam) + Va*cos(gam)*(dWx_dz*cos(chi)*cos(gam) - dWz_dz*sin(gam) + dWy_dz*cos(gam)*sin(chi)) + Va*cos(chi)*sin(gam)*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)) + Va*sin(chi)*sin(gam)*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)));
			tabG[5] 	= dt*((dWy_dx*cos(chi)*cos(gam) - dWx_dx*cos(gam)*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi)*cos(gam) - dWx_dy*cos(gam)*sin(chi)) + (dWy_dz*cos(chi)*cos(gam) - dWx_dz*cos(gam)*sin(chi))*(Wz - Va*sin(gam)) + Va*cos(chi)*cos(gam)*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)) - Va*cos(gam)*sin(chi)*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)));
			tabG[7] 	= (CL*rho*ac.SS*(Va*Va)*dt)/(ac.AR*M_PI*ac.ee*ac.mm);
			tabG[10] 	= -dt/ac.mm;
			tabG[11]    = (Wx + Va*cos(chi)*cos(gam))*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)) + (Wy + Va*cos(gam)*sin(chi))*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)) + (Wz - Va*sin(gam))*(dWx_dz*cos(chi)*cos(gam) - dWz_dz*sin(gam) + dWy_dz*cos(gam)*sin(chi)) - T/ac.mm + g*sin(gam) + (rho*ac.SS*(Va*Va)*(ac.Cd0 + (CL*CL)/(ac.AR*M_PI*ac.ee)))/(2.0*ac.mm);

			break;

		//----------------------------------------
		// F5
		case 5:

			//-------
			// Tabulated G
			tabG[3] 	= (dt*((Wx + Va*cos(chi)*cos(gam))*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) + (Wz - Va*sin(gam))*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) - g*cos(gam) + (CL*rho*ac.SS*(Va*Va)*cos(phi))/(2.0*ac.mm)))/(Va*Va) - (dt*(cos(chi)*cos(gam)*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) - sin(gam)*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) + cos(gam)*sin(chi)*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) + (CL*rho*ac.SS*Va*cos(phi))/ac.mm))/Va;
			tabG[4] 	= - (dt*((Wx + Va*cos(chi)*cos(gam))*(dWx_dx*cos(chi)*cos(gam) - dWz_dx*sin(gam) + dWy_dx*cos(gam)*sin(chi)) + (Wy + Va*cos(gam)*sin(chi))*(dWx_dy*cos(chi)*cos(gam) - dWz_dy*sin(gam) + dWy_dy*cos(gam)*sin(chi)) + (Wz - Va*sin(gam))*(dWx_dz*cos(chi)*cos(gam) - dWz_dz*sin(gam) + dWy_dz*cos(gam)*sin(chi)) + g*sin(gam) - Va*cos(gam)*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) - Va*cos(chi)*sin(gam)*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) - Va*sin(chi)*sin(gam)*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam))))/Va - 1.0;
			tabG[5] 	= -(dt*((dWy_dx*cos(chi)*sin(gam) - dWx_dx*sin(chi)*sin(gam))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi)*sin(gam) - dWx_dy*sin(chi)*sin(gam)) + (dWy_dz*cos(chi)*sin(gam) - dWx_dz*sin(chi)*sin(gam))*(Wz - Va*sin(gam)) + Va*cos(chi)*cos(gam)*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) - Va*cos(gam)*sin(chi)*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam))))/Va;
			tabG[6] 	= (CL*rho*ac.SS*Va*dt*sin(phi))/(2.0*ac.mm);
			tabG[7] 	= -(rho*ac.SS*Va*dt*cos(phi))/(2.0*ac.mm);
			tabG[11] 	= -((Wx + Va*cos(chi)*cos(gam))*(dWz_dx*cos(gam) + dWx_dx*cos(chi)*sin(gam) + dWy_dx*sin(chi)*sin(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWz_dy*cos(gam) + dWx_dy*cos(chi)*sin(gam) + dWy_dy*sin(chi)*sin(gam)) + (Wz - Va*sin(gam))*(dWz_dz*cos(gam) + dWx_dz*cos(chi)*sin(gam) + dWy_dz*sin(chi)*sin(gam)) - g*cos(gam) + (CL*rho*ac.SS*(Va*Va)*cos(phi))/(2.0*ac.mm))/Va;

			break;

		//----------------------------------------
		// F6
		case 6:

			//-------
			// Tabulated G
			tabG[3] 	= - (dt*(sin(gam)*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) - cos(chi)*cos(gam)*(dWy_dx*cos(chi) - dWx_dx*sin(chi)) - cos(gam)*sin(chi)*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) + (CL*rho*ac.SS*Va*sin(phi))/ac.mm))/(Va*cos(gam)) - (dt*((Wz - Va*sin(gam))*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) + (dWy_dx*cos(chi) - dWx_dx*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) - (CL*rho*ac.SS*(Va*Va)*sin(phi))/(2.0*ac.mm)))/((Va*Va)*cos(gam));
			tabG[4] 	= (dt*sin(gam)*((Wz - Va*sin(gam))*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) + (dWy_dx*cos(chi) - dWx_dx*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) - (CL*rho*ac.SS*(Va*Va)*sin(phi))/(2.0*ac.mm)))/(Va*(cos(gam)*cos(gam))) - (dt*(Va*cos(gam)*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) + Va*cos(chi)*sin(gam)*(dWy_dx*cos(chi) - dWx_dx*sin(chi)) + Va*sin(chi)*sin(gam)*(dWy_dy*cos(chi) - dWx_dy*sin(chi))))/(Va*cos(gam));
			tabG[5] 	= - (dt*((Wz - Va*sin(gam))*(dWx_dz*cos(chi) + dWy_dz*sin(chi)) + (dWx_dx*cos(chi) + dWy_dx*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWx_dy*cos(chi) + dWy_dy*sin(chi)) - Va*cos(chi)*cos(gam)*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) + Va*cos(gam)*sin(chi)*(dWy_dx*cos(chi) - dWx_dx*sin(chi))))/(Va*cos(gam)) - 1.0;
			tabG[6] 	= -(CL*rho*ac.SS*Va*dt*cos(phi))/(2.0*ac.mm*cos(gam));
			tabG[7] 	= -(rho*ac.SS*Va*dt*sin(phi))/(2.0*ac.mm*cos(gam));
			tabG[11] 	= ((Wz - Va*sin(gam))*(dWy_dz*cos(chi) - dWx_dz*sin(chi)) + (dWy_dx*cos(chi) - dWx_dx*sin(chi))*(Wx + Va*cos(chi)*cos(gam)) + (Wy + Va*cos(gam)*sin(chi))*(dWy_dy*cos(chi) - dWx_dy*sin(chi)) - (CL*rho*ac.SS*(Va*Va)*sin(phi))/(2.0*ac.mm))/(Va*cos(gam));

			break;

		//----------------------------------------
		// F7
		case 7:

			//-------
			// Tabulated G
			tabG[6] 	= -1.0;
			tabG[8] 	= -dt;
			tabG[11] 	= -dphi;

			break;

		//----------------------------------------
		// F8
		case 8:

			//-------
			// Tabulated G
			tabG[7] 	= -1.0;
			tabG[9] 	= -dt;
			tabG[11] 	= -dCL;

			break;

		//------
		// Default
		default:
			break;
		}

		//-------------------
		// Define Gs

		Gs = tabG[xnum];
		Gnonzero = 1;
	}
	else if ( (tx == (tf+1)) && (xnum == (Fnum-1)) && (Fnum <= sn.numstates) )
	{
		//-------
		// Account for x+1 state in F
		Gs = 1.0;
		Gnonzero = 1;
	}
	return Gs;
}

/**
 * Initiates SNOPT algorithm
 *
 */
void problem::runSNOPT()
{
	//Report TOL status
	cout << "TOL STATUS: Solving now" << endl;

	// Develop optimization problem
	snoptProblemA GeneralWindProb;

	// Load SNOPT variables into optimization problem
	GeneralWindProb.setNeA        ( neA );								// Not sure what neA is
	GeneralWindProb.setNeG        ( neG );								// Set number of non-zero Jacobian entries
	GeneralWindProb.setIntParameter("Derivative option",1 );			// Set derivative option (i.e 0 if no Jacobians given, 1 if Jacobians are provided by the user)
	GeneralWindProb.setPrintFile  ( "GeneralWind.out" );				// Set print file
	GeneralWindProb.setProblemSize( n, neF );                         	// Set number of states and number of constraints
	GeneralWindProb.setObjective  ( ObjRow, ObjAdd );                 	// Set objective function (cost function)
	GeneralWindProb.setA          ( lenA, iAfun, jAvar, A );			// Not sure what A is
	GeneralWindProb.setG          ( lenG, iGfun, jGvar );               // Set Jacobian length and indices
	GeneralWindProb.setX          ( x, xlow, xupp, xmul, xstate );		// Set States and Limits
	GeneralWindProb.setF          ( F, Flow, Fupp, Fmul, Fstate );		// Set Constraints and Limits
	GeneralWindProb.setProbName   ( "GeneralWindOpt1" );				// Set problem name
	GeneralWindProb.setUserFun    ( DEFINEGusrfg_ );					// Set function to be called by SNOPT
	GeneralWindProb.setRealParameter( "Major optimality tolerance", sn.opt_tol);	//Set Optimality Tolerance
	GeneralWindProb.setRealParameter( "Major feasibility tolerance", sn.feas_tol);	//Set Feasibility Tolerance
	GeneralWindProb.setIntParameter( "Iterations limit", 60000);		// Set minor iteration limit
	int Cold = 0;                           							// Set solver to cold (no previous data to bootstrap)
	GeneralWindProb.solve( Cold );										// Run SNOPT (Problem A Type)
}

/**
 * Writes optimized state to a json file
 *
 * @param[in] filename JSON filename to be written
 */
void problem::writeJSON(string filename)
{
	//JSON Document
	Json::Value snopt_results;

	//JSON arrays
	Json::Value leg1;

	Json::Value aircraft;
	Json::Value gains;
	Json::Value limits;
	Json::Value snopt;

	Json::Value trajectory;
	Json::Value args;
	Json::Value time_arr;
	Json::Value xs;
	Json::Value ys;
	Json::Value zs;
	Json::Value Va;
	Json::Value gam;
	Json::Value chi;
	Json::Value phi;
	Json::Value CL;
	Json::Value dphi;
	Json::Value dCL;
	Json::Value T;

	double times = 0;
	for (int ii = 0; ii < (sn.ts+1); ii++)
	{
		time_arr.append(times);
		xs.append(x[1+ii*(sn.numinp)]);
		ys.append(x[2+ii*(sn.numinp)]);
		zs.append(x[3+ii*(sn.numinp)]);
		Va.append(x[4+ii*(sn.numinp)]);
		gam.append(x[5+ii*(sn.numinp)]);
		chi.append(x[6+ii*(sn.numinp)]);
		phi.append(x[7+ii*(sn.numinp)]);
		CL.append(x[8+ii*(sn.numinp)]);
		dphi.append(x[9+ii*(sn.numinp)]);
		dCL.append(x[10+ii*(sn.numinp)]);
		T.append(x[11+ii*(sn.numinp)]);
		times = times + x[0]; //iterate time
	}

	snopt_results["args"]["east"] = east;
	snopt_results["args"]["north"] = north;
	snopt_results["args"]["up"] = up;
	snopt_results["args"]["xg"] = xg;
	snopt_results["args"]["yg"] = yg;
	snopt_results["args"]["zg"] = zg;
	snopt_results["args"]["rd"] = rg;
	snopt_results["args"]["aircraft"] = aircraft_type;
	snopt_results["args"]["problem"] = mission;

	snopt_results["problem"] = mission;

	snopt_results["FinalCost"] = F[0];
	snopt_results["dt"] = x[0];
	snopt_results["trajectory"]["time"] = time_arr;
	snopt_results["trajectory"]["x"] = xs;
	snopt_results["trajectory"]["y"] = ys;
	snopt_results["trajectory"]["z"] = zs;
	snopt_results["trajectory"]["Va"] = Va;
	snopt_results["trajectory"]["gam"] = gam;
	snopt_results["trajectory"]["chi"] = chi;
	snopt_results["trajectory"]["phi"] = phi;
	snopt_results["trajectory"]["CL"] = CL;
	snopt_results["trajectory"]["dphi"] = dphi;
	snopt_results["trajectory"]["dCL"] = dCL;
	snopt_results["trajectory"]["T"] = T;

	snopt_results["aircraft"]["name"] = aircraft_type;
	snopt_results["aircraft"]["mass"] = ac.mm;
	snopt_results["aircraft"]["b"] = ac.b;
	snopt_results["aircraft"]["S"] = ac.SS;
	snopt_results["aircraft"]["e"] = ac.ee;
	snopt_results["aircraft"]["AR"] = ac.AR;
	snopt_results["aircraft"]["Cd0"] = ac.Cd0;
	snopt_results["aircraft"]["CLmin"] = ac.CLmin;
	snopt_results["aircraft"]["CLmax"] = ac.CLmax;
	snopt_results["aircraft"]["phimax"] = ac.phimax;
	snopt_results["aircraft"]["Vamin"] = ac.Vamin;
	snopt_results["aircraft"]["Vamax"] = ac.Vamax;
	snopt_results["aircraft"]["gammamax"] = ac.gammamax;
	snopt_results["aircraft"]["dphimax"] = ac.phidotmax;
	snopt_results["aircraft"]["Tmin"] = ac.Tmin;
	snopt_results["aircraft"]["Tmax"] = ac.Tmax;

	snopt_results["gains"]["kT"] = gn.kT;
	snopt_results["gains"]["kp"] = gn.kp;
	snopt_results["gains"]["kv"] = gn.kv;
	snopt_results["gains"]["ka"] = gn.ka;
	snopt_results["gains"]["kdt"] = gn.kdt;

	snopt_results["limits"]["dtmin"] = lm.dtmin;
	snopt_results["limits"]["dtmax"] = lm.dtmax;
	snopt_results["limits"]["xmin"] = lm.xmin;
	snopt_results["limits"]["xmax"] = lm.xmax;
	snopt_results["limits"]["ymin"] = lm.ymin;
	snopt_results["limits"]["ymax"] = lm.ymax;
	snopt_results["limits"]["zmin"] = lm.zmin;
	snopt_results["limits"]["zmax"] = lm.zmax;

	snopt_results["snopt"]["ts"] = sn.ts;
	snopt_results["snopt"]["numinp"] = sn.numinp;
	snopt_results["snopt"]["numstates"] = sn.numstates;
	snopt_results["snopt"]["numbounds"] = sn.numbounds;
	snopt_results["snopt"]["opt_tol"] = sn.opt_tol;
	snopt_results["snopt"]["feas_tol"] = sn.feas_tol;

	// write in a nice readable way
	Json::StyledWriter styledWriter;
	ofstream jsonfile;
	jsonfile.open(filename);
	jsonfile << styledWriter.write(snopt_results);
	jsonfile.close();
}

/**
 * Writes optimized state to a text file
 *
 */
void problem::writeTXT(string filename)
{
	//Create/overwrite file
	FILE* txtfile;
	txtfile = fopen("snopt_output.txt","w");

	double time_s;
	time_s = 0.0;

	//added by Will
	double tfinal = 10;
	double dt = tfinal/sn.ts;

	fprintf(txtfile, "%% SNOPT Output: Thesis Optimization \n");
	fprintf(txtfile, "%% Simulation: tf_i = %4.2f s, dt_i = %4.2f s \n",tfinal,dt);
	fprintf(txtfile, "%% ");
	fprintf(txtfile, "time \t \t");
	fprintf(txtfile, "x \t \t");    fprintf(txtfile, "y \t \t");
	fprintf(txtfile, "z \t \t");    fprintf(txtfile, "Va \t \t");
	fprintf(txtfile, "gamma \t \t");fprintf(txtfile, "chi \t \t");
	fprintf(txtfile, "phi \t \t");  fprintf(txtfile, "CL \t \t");
	fprintf(txtfile, "dphi \t \t"); fprintf(txtfile, "dCL \t \t");
	fprintf(txtfile, "T \t \t");    fprintf(txtfile, "dt \t \t");
	fprintf(txtfile, "Final Cost \n");
	fprintf(txtfile, "ProblemS10 \n");

	for (int ii = 0; ii <= (sn.ts); ii++)
	{
		fprintf(txtfile, "%-4.7e \t",time_s);
		fprintf(txtfile, "%-4.7e \t",x[1+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[2+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[3+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[4+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[5+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[6+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[7+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[8+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[9+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[10+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[11+ii*sn.numinp]);
		fprintf(txtfile, "%-4.7e \t",x[0]);
		fprintf(txtfile, "%-4.7e \n",F[0]);
		time_s = time_s + x[0];
	}

	//Close text file
	fclose (txtfile);
}

/**
 * Problem object destructor.
 * Destroys all dynamically allocated variables.
 */
problem::~problem()
{
	  delete []iAfun;  delete []jAvar;  delete []A;
	  delete []iGfun;  delete []jGvar;

	  delete []x;      delete []xlow;   delete []xupp;
	  delete []xmul;   delete []xstate;

	  delete []F;      delete []Flow;   delete []Fupp;
	  delete []Fmul;   delete []Fstate;

	  delete []xnames; delete []Fnames;
}
