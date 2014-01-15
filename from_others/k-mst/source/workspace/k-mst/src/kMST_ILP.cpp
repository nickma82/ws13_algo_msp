#include "kMST_ILP.h"
#include <sstream>
#include <fstream>

kMST_ILP::kMST_ILP( Instance& _instance, string _model_type, int _k ) :
		instance( _instance ), model_type( _model_type ), k( _k )
{
	n = instance.n_nodes;
	m = instance.n_edges;
	if( k == 0 ) k = n;
	W = instance.edges[instance.incidentEdges[0].front()].weight;
}

void kMST_ILP::solve()
{


	try {
		// initialize CPLEX
		env = IloEnv();
		model = IloModel( env );

		// add model-specific constraints
		if( model_type == "scf" ) modelSCF();
		else if( model_type == "mcf" ) modelMCF();
		else if( model_type == "mtz" ) modelMTZ();
		else {
			cerr << "No existing model chosen" << endl;
			exit( -1 );
		}

//		// build model
//		cplex = IloCplex( model );
//		// export model to a text file
//		//cplex.exportModel( "model.lp" );
//		// set parameters
//		setCPLEXParameters();
//
//		// solve model
//		cout << "Calling CPLEX solve ..." << endl;
//		cplex.solve();
//		cout << "CPLEX finished." << endl << endl;
//		cout << "CPLEX status: " << cplex.getStatus() << endl;
//		cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
//		cout << "Objective value: " << cplex.getObjValue() << endl;
//		cout << "CPU time: " << Tools::CPUtime() << endl;
//		cout << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;
//
//		cplex.exportModel("lp/kmst.lp");

	}
	catch( IloException& e ) {
		cerr << "kMST_ILP: exception " << e << endl;
		exit( -1 );
	}
	catch( ... ) {
		cerr << "kMST_ILP: unknown exception." << endl;
		exit( -1 );
	}
}

// ----- private methods -----------------------------------------------

void kMST_ILP::setCPLEXParameters()
{
	// print every x-th line of node-log and give more details
	cplex.setParam( IloCplex::MIPInterval, 1 );
	cplex.setParam( IloCplex::MIPDisplay, 2 );
	// deactivate CPLEX general-purpose cuts (seemed to be faster here)
	cplex.setParam( IloCplex::EachCutLim, 0 );
	cplex.setParam( IloCplex::FracCuts, -1 );
	// only use a single thread
	cplex.setParam( IloCplex::Threads, 1 );
}

void kMST_ILP::modelSCF()
{
	list<u_int>::iterator it;

	fstream file_dat;
	file_dat.open("test.dat", ios::out);

	// (12) // Initialization of used edges
	IloBoolVarArray x(env, 2*m);
	char VarName[8];
	for(u_int e=0; e < 2*m; e++)
	{
		sprintf( VarName, "x_%d",e);
		x[e] = IloBoolVar(env,VarName);
	}

	// (1) // objective function
	IloExpr expr(env);
	for(u_int e=n-1; e < m; e++)
	{
		expr += instance.edges.at(e).weight * x[e];
		expr += instance.edges.at(e).weight * x[e+m];
	}
	model.add(IloMinimize(env, expr));  //let CPLEX minimize the objective function
	expr.end();

	// (11) // Initialization of flow variable
	IloIntVarArray f(env, 2*m);
	for(u_int e=0; e < 2*m; e++)
	{
		sprintf( VarName, "f_%d",e);
		f[e] = IloIntVar(env, 0, k, VarName);
	}

	// (13) // Initialization of y (support array to include k+1 nodes (with artificial node))
	IloBoolVarArray y(env, n);
	for(u_int e=0; e < n; e++)
	{
		sprintf( VarName, "y_%d",e);
		y[e] = IloBoolVar(env,VarName);
	}

	// (7) (8) // if an edge x_ij is selected -> y_i and y_j must be 1
	for(u_int e=0; e < 2*m; e++)
	{
		IloNumExpr Expr40(env);
		IloNumExpr Expr41(env);
		Expr40 += x[e];
		Expr41 += x[e];
		model.add(Expr40 <= y[instance.edges.at(e%m).v1]);
		model.add(Expr41 <= y[instance.edges.at(e%m).v2]);
		Expr40.end();
		Expr41.end();
	}

	// (10) // exactly k+1 different nodes allowed (with artificial node)
	IloNumExpr Expr34(env);
	for(u_int v=0; v < n; v++)
	{
		Expr34 += y[v];
	}
	model.add(Expr34 == k+1);
	Expr34.end();

	// (9) //
	for(u_int e=0; e < m; e++)
	{
		IloNumExpr Expr50(env);
		Expr50 += y[instance.edges.at(e).v1];
		Expr50 += x[e];
		Expr50 += x[e+m];

		model.add(Expr50 <= y[instance.edges.at(e).v2] + 1);
		Expr50.end();

		IloNumExpr Expr51(env);
		Expr51 += y[instance.edges.at(e).v2];
		Expr51 += x[e];
		Expr51 += x[e+m];

		model.add(Expr51 <= y[instance.edges.at(e).v1] + 1);
		Expr51.end();
	}


	// (2) // Flow expression from node 0 (flow k outgoing)
	IloNumExpr Expr2(env);
	for(it=instance.incidentEdges.at(0).begin(); it != instance.incidentEdges.at(0).end(); it++)
	{
		if(instance.edges.at(*it).v1 == 0)
		{	// outgoing edge
			Expr2 += f[*it];
			Expr2 -= f[(*it)+m];
		}
		else
		{	// incoming edge
			Expr2 -= f[*it];
			Expr2 += f[(*it)+m];
		}

	}
	model.add(Expr2 == k);
	Expr2.end();


	// (4) // check flow
	for(u_int v=1; v < n; v++)
	{
		IloNumExpr Expr3(env);
		IloNumExpr Expr3_right(env);

		for(it=instance.incidentEdges.at(v).begin(); it != instance.incidentEdges.at(v).end(); it++)
		{
			if(instance.edges.at(*it).v1 == v)
			{	// outgoing edge
				Expr3 -= f[*it];
				Expr3 += f[(*it)+m];

				Expr3_right += f[(*it)+m];
			}
			else
			{	// incoming edge
				Expr3 += f[*it];
				Expr3 -= f[(*it)+m];

				Expr3_right += f[*it];
			}
		}
		model.add(Expr3 == IloMin(1,Expr3_right));

		Expr3.end();
		Expr3_right.end();
	}

	// (5) // force x[e] to be set if flow exist
	for(u_int e=0; e < 2*m; e++)
	{
		IloNumExpr Expr4(env);
		Expr4 += f[e];
		model.add(Expr4 <= (k * x[e]));
		Expr4.end();
	}

	// (3) //
	IloNumExpr Expr5(env);
	for(u_int e=n-1; e < m; e++)
	{
		Expr5 += x[e];
		Expr5 += x[e+m];
	}
	model.add(Expr5 == k-1);
	Expr5.end();

	// (6) // force the flow from node 0 to flow over exactly one edge
	IloNumExpr Expr6(env);
	for(u_int e=0; e < n-1; e++)
	{
		Expr6 += x[e];
	}
	model.add(Expr6 == 1);
	Expr6.end();

	// build model
	cplex = IloCplex( model );
	// export model to a text file
	//cplex.exportModel( "model.lp" );
	// set parameters
	setCPLEXParameters();

	// solve model
	cout << "Calling CPLEX solve ..." << endl;
	cplex.solve();
	cout << "CPLEX finished." << endl << endl;
	cout << "CPLEX status: " << cplex.getStatus() << endl;
	cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	cout << "Objective value: " << cplex.getObjValue() << endl;
	cout << "CPU time: " << Tools::CPUtime() << endl;
	cout << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;

	file_dat << "CPLEX finished." << endl << endl;
	file_dat << "CPLEX status: " << cplex.getStatus() << endl;
	file_dat << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	file_dat << "Objective value: " << cplex.getObjValue() << endl;
	file_dat << "CPU time: " << Tools::CPUtime() << endl;
	file_dat << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;
	file_dat.close();
}




void kMST_ILP::modelMCF()
{
	list<u_int>::iterator it;

	fstream file_dat;
	file_dat.open("test.dat", ios::out);

	// (27) // Initialization of used edges
	IloBoolVarArray x(env, 2*m);
	char VarName[12];
	for(u_int e=0; e < 2*m; e++)
	{
		sprintf( VarName, "x_%d",e);
		x[e] = IloBoolVar(env,VarName);
	}

	// (14) // objective function
	IloExpr expr(env);
	for(u_int e=n-1; e < m; e++)
	{
		expr += instance.edges.at(e).weight * x[e];
		expr += instance.edges.at(e).weight * x[e+m];
	}
	model.add(IloMinimize(env, expr));
	expr.end();

  // Initialization of f ... size: |V| * |E|
  IloNumVarArray f(env, 2*m*(n-1));
  for(u_int l=0; l < n-1; l++)
  {
    for(u_int e=0; e < 2*m; e++)
    {
        sprintf( VarName, "f_%d_%d",e,l);
        f[(2 * m * l) + e] = IloNumVar(env,0,1,VarName);
     }
  }

	// (28) // Initialization of y (support array to include k+1 nodes (with artificial node))
	IloBoolVarArray y(env, n);
	for(u_int e=0; e < n; e++)
	{
		sprintf( VarName, "y_%d",e);
		y[e] = IloBoolVar(env,VarName);
	}

	// (22) (23) // if an edge x_ij is selected -> y_i and y_j must be 1
	for(u_int e=0; e < 2*m; e++)
	{
		IloNumExpr Expr40(env);
		IloNumExpr Expr41(env);
		Expr40 += x[e];
		Expr41 += x[e];
		model.add(Expr40 <= y[instance.edges.at(e%m).v1]);
		model.add(Expr41 <= y[instance.edges.at(e%m).v2]);
		Expr40.end();
		Expr41.end();
	}

	// (25) // exactly k+1 different nodes allowed (with artificial node)
	IloNumExpr Expr34(env);
	for(u_int v=0; v < n; v++)
	{
		Expr34 += y[v];
	}
	model.add(Expr34 == k+1);
	Expr34.end();

	// (24) //
	for(u_int e=0; e < m; e++)
	{
		IloNumExpr Expr50(env);
		Expr50 += y[instance.edges.at(e).v1];
		Expr50 += x[e];
		Expr50 += x[e+m];

		model.add(Expr50 <= y[instance.edges.at(e).v2] + 1);
		Expr50.end();

		IloNumExpr Expr51(env);
		Expr51 += y[instance.edges.at(e).v2];
		Expr51 += x[e];
		Expr51 += x[e+m];

		model.add(Expr51 <= y[instance.edges.at(e).v1] + 1);
		Expr51.end();
	}


	// (20) //
	for(u_int l=0; l < n-1; l++)
	{
	 IloNumExpr Expr_left(env);
	 for(it=instance.incidentEdges.at(0).begin(); it != instance.incidentEdges.at(0).end(); it++)
	 {
		if(instance.edges.at(*it).v1 == 0)
		{	// outgoing edge
			Expr_left += f[(2*m*l) + (*it)];
			Expr_left -= f[(2*m*l) + ((*it) + m)];
		}
	 }

	 IloNumExpr Expr_right(env);
	 for(it=instance.incidentEdges.at(l+1).begin(); it != instance.incidentEdges.at(l+1).end(); it++)
	 {
		if(instance.edges.at(*it).v2 == l+1)
		{	// incoming edge normal
			Expr_right += f[(2*m*l) + (*it)];
		}
		else
		{	// incoming artificial edge
			Expr_right += f[(2*m*l) + ((*it) + m)];
		}
	 }

	 model.add(Expr_left == Expr_right);
	 Expr_left.end();
	 Expr_right.end();
	}

	// (15) //
	for(u_int l=0; l < n-1; l++)
	{
	 IloNumExpr Expr2(env);
	 for(it=instance.incidentEdges.at(0).begin(); it != instance.incidentEdges.at(0).end(); it++)
	 {
		if(instance.edges.at(*it).v1 == 0)
		{	// outgoing edge
		   Expr2 += f[(2*m*l) + (*it)];
		   Expr2 -= f[(2*m*l) + ((*it) + m)];
		}
	 }
	 model.add(Expr2 <= 1);
	 Expr2.end();
	}

	// (16) //
	IloNumExpr Expr3(env);
	for(u_int l=0; l < n-1; l++)
	{
	 for(it=instance.incidentEdges.at(0).begin(); it != instance.incidentEdges.at(0).end(); it++)
	 {
		if(instance.edges.at(*it).v1 == 0)
		{	// outgoing edge
		   Expr3 += f[(2*m*l) + (*it)];
		   Expr3 -= f[(2*m*l) + ((*it) + m)];
		}
	 }
	}
	model.add(Expr3 == k);
	Expr3.end();


	// (17) //
	for(u_int l=0; l < n-1; l++)
	{
	 IloNumExpr Expr4(env);
	 for(it=instance.incidentEdges.at(l+1).begin(); it != instance.incidentEdges.at(l+1).end(); it++)
	 {
		if(instance.edges.at(*it).v2 == l+1)
		{	// incoming edge normal
		   Expr4 += f[(2*m*l) + (*it)];
		}
		else
		{	// incoming artificial edge
		   Expr4 += f[(2*m*l) + ((*it) + m)];
		}
	 }
	 model.add(Expr4 <= 1);
	 Expr4.end();
	}

	// (18) //
	IloNumExpr Expr5(env);
	for(u_int l=0; l < n-1; l++)
	{
	 for(it=instance.incidentEdges.at(l+1).begin(); it != instance.incidentEdges.at(l+1).end(); it++)
	 {
		if(instance.edges.at(*it).v2 == l+1)
		{	// incoming edge normal
		   Expr5 += f[(2*m*l) + (*it)];
		}
		else
		{	// incoming artificial edge
		   Expr5 += f[(2*m*l) + ((*it) + m)];
		}
	 }
	}
	model.add(Expr5 == k);
	Expr5.end();

	// (19) //
	for(u_int l=0; l < n-1; l++)
	{
	  for(u_int j=1; j < n; j++)
	  {
		  if(j == l+1)
			  continue;

		  IloNumExpr Expr6(env);

		  for(it=instance.incidentEdges.at(j).begin(); it != instance.incidentEdges.at(j).end(); it++)
		  {
			 if(instance.edges.at(*it).v2 == j)
			 {

				Expr6 += f[(2*m*l) + (*it)];
				Expr6 -= f[(2*m*l) + (*it + m)];

			 }
			 else if((instance.edges.at(*it).v1 == j))
			 {

				Expr6 -= f[(2*m*l) + (*it)];
				Expr6 += f[(2*m*l) + (*it + m)];

			 }
		  }

		  model.add(Expr6 == 0);
		  Expr6.end();
	  }
	}

	// (26) // force x[e] to be set if flow exist
	for(u_int l=0; l < n-1; l++)
	{
	 for(u_int e=0; e < 2*m; e++)
	 {
		IloNumExpr Expr7(env);
		Expr7 += f[(2*l*m) + e];
		model.add(Expr7 <= x[e]);
		Expr7.end();
	 }
	}

	//
	IloNumExpr Expr8(env);
	for(u_int e=n-1; e < m; e++)
	{
		Expr8 += x[e];
		Expr8 += x[e+m];
	}
	model.add(Expr8 == k-1);
	Expr8.end();


	// (21) //
	IloIntExpr Expr9(env);
	for(u_int e=0; e < n-1; e++)
	{
		Expr9 += x[e];
	}
	model.add(Expr9 == 1);
	Expr9.end();


	// build model
	cplex = IloCplex( model );
	// export model to a text file
	//cplex.exportModel( "model.lp" );
	// set parameters
	setCPLEXParameters();

	// solve model
	cout << "Calling CPLEX solve ..." << endl;
	cplex.solve();


//	for(u_int i=0; i<m; i++)
//	{
//		cout << "x[" << i << "] = " << cplex.getValue(x[i]) << endl;
//	}
//
//	for(u_int l=0; l < n-1; l++)
//	{
//		for(u_int e=0; e < 2*m; e++)
//		{
//			cout << "f_" << e <<"_" << l  << " = " << cplex.getValue(f[(2 * m * l) + e]) << endl;
//		 }
//	}

	cout << "CPLEX finished." << endl << endl;
	cout << "CPLEX status: " << cplex.getStatus() << endl;
	cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	cout << "Objective value: " << cplex.getObjValue() << endl;
	cout << "CPU time: " << Tools::CPUtime() << endl;
	cout << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;

	file_dat << "CPLEX finished." << endl << endl;
	file_dat << "CPLEX status: " << cplex.getStatus() << endl;
	file_dat << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	file_dat << "Objective value: " << cplex.getObjValue() << endl;
	file_dat << "CPU time: " << Tools::CPUtime() << endl;
	file_dat << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;
	file_dat.close();
}

void kMST_ILP::modelMTZ()
{
	list<u_int>::iterator it;

	fstream file_dat;
	file_dat.open("test.dat", ios::out);

	// (42) // Initialization of x (array of used edges in the solution)
	IloBoolVarArray x(env, 2*m);
	char VarName[24];
	for(u_int e=0; e < 2*m; e++)
	{
		sprintf( VarName, "x_%d",e);
		x[e] = IloBoolVar(env,VarName);
	}

	// Initialization of u (array of used nodes in the solution)
	IloIntVarArray u(env, n);
	for(u_int e=0; e < n; e++)
	{
		sprintf( VarName, "u_%d",e);
		u[e] = IloIntVar(env,0, k,VarName);
	}

	// (43) // Initialization of y (support array to include only k nodes)
	IloBoolVarArray y(env, n);
	for(u_int e=0; e < n; e++)
	{
		sprintf( VarName, "y_%d",e);
		y[e] = IloBoolVar(env,VarName);
	}

	// (29 // objective function
	IloExpr expr(env);
	for(u_int e=n-1; e < m; e++)
	{
		expr += instance.edges.at(e).weight * x[e];
		expr += instance.edges.at(e).weight * x[e+m];
	}
	model.add(IloMinimize(env, expr));
	expr.end();

	// (31) // u[v] has to be between 0 and k
	for(u_int v=1; v < n; v++)
	{
		IloNumExpr Expr2(env);
		Expr2 += u[v];
		model.add(Expr2 <= k);
		Expr2.end();
	}

	// (32) // u[0] must be 0
	IloNumExpr Expr5(env);
	Expr5 += u[0];
	model.add(Expr5 == 0);
	Expr5.end();

	// (33) // exactly 1 outgoing edge from the root
	IloNumExpr Expr4(env);
	for(u_int e=0; e < n-1; e++)
	{
		Expr4 += x[e];
	}
	model.add(Expr4 == 1);
	Expr4.end();

	// (34) // k-1 edges are allowed in the solution
	IloNumExpr Expr6(env);
	for(u_int e=n-1; e < m; e++)
	{
		Expr6 += x[e];
		Expr6 += x[e+m];
	}
	model.add(Expr6 == k-1);
	Expr6.end();

	// (35) // sum of the u value
	IloNumExpr Expr7(env);
	for(u_int v=0; v < n; v++)
	{
		Expr7 += u[v];
	}
	model.add(Expr7 == (k*(k+1))/2);
	Expr7.end();

	// (36) // force y to be 1 if u is in the solution
	for(u_int v=0; v < n; v++)
	{
		IloNumExpr Expr33(env);
		Expr33 += u[v];
		model.add(Expr33 <= k*y[v]);
		Expr33.end();
	}

	// (37) // only k different nodes allowed
	IloNumExpr Expr34(env);
	for(u_int v=0; v < n; v++)
	{
		Expr34 += y[v];
	}
	model.add(Expr34 == k);
	Expr34.end();

	// (39) (40) // if f_ij in the solution -> u[i] and u[j] must be greater than 0
	for(u_int v=1; v < n; v++)
	{
		for(it=instance.incidentEdges.at(v).begin(); it != instance.incidentEdges.at(v).end(); it++)
		{
			if(instance.edges.at(*it).v1 == 0)
			{
				IloNumExpr Expr6(env);
				IloNumExpr Expr7(env);
				Expr6 += x[(*it)];
				Expr7 += x[(*it)+m];
				model.add(Expr6 <= u[instance.edges.at(*it).v2]);
				model.add(Expr7 <= u[instance.edges.at(*it).v2]);
				Expr6.end();
				Expr7.end();
			}
			else
			{
				IloNumExpr Expr6(env);
				IloNumExpr Expr7(env);
				Expr6 += x[(*it)];
				Expr7 += x[(*it)];
				model.add(Expr6 <= u[instance.edges.at(*it).v1]);
				model.add(Expr7 <= u[instance.edges.at(*it).v2]);
				Expr6.end();
				Expr7.end();

				IloNumExpr Expr40(env);
				IloNumExpr Expr41(env);
				Expr40 += x[(*it)+m];
				Expr41 += x[(*it)+m];
				model.add(Expr40 <= u[instance.edges.at(*it).v1]);
				model.add(Expr41 <= u[instance.edges.at(*it).v2]);
				Expr40.end();
				Expr41.end();
			}
		}
	}

	// (30) // miller tucker zemlin formulation
	for(u_int e=0; e < m; e++)
	{
		IloNumExpr Expr4(env);
		Expr4 += u[instance.edges.at(e).v1];
		Expr4 += x[e];

		model.add(Expr4 <= u[instance.edges.at(e).v2] + k*(1-x[e]));
		Expr4.end();

		IloNumExpr Expr12(env);
		Expr12 += u[instance.edges.at(e).v2];
		Expr12 += x[(e)+m];

		model.add(Expr12 <= u[instance.edges.at(e).v1] + k*(1-x[(e)+m]));
		Expr12.end();
	}

	// (41) //
	for(u_int e=n-1; e < m; e++)
	{
		IloNumExpr Expr50(env);
		Expr50 += y[instance.edges.at(e).v1];
		Expr50 += x[e];
		Expr50 += x[e+m];

		model.add(Expr50 <= y[instance.edges.at(e).v2] + 1);
		Expr50.end();

		IloNumExpr Expr51(env);
		Expr51 += y[instance.edges.at(e).v2];
		Expr51 += x[e];
		Expr51 += x[e+m];

		model.add(Expr51 <= y[instance.edges.at(e).v1] + 1);
		Expr51.end();
	}

	// (38) // every node \ 0 must have an incoming flow of <= 1
	for(u_int v=1; v < n; v++)
	{
		IloNumExpr Expr11(env);

		for(it=instance.incidentEdges.at(v).begin(); it != instance.incidentEdges.at(v).end(); it++)
		{
			if(instance.edges.at(*it).v2 == v)
			{	// incoming edge
				Expr11 += x[*it];
			}
			else
			{	// outgoing edge
				Expr11 += x[(*it)+m];
			}
		}
		model.add(Expr11 <= 1);

		Expr11.end();
	}

	// build model
	cplex = IloCplex( model );
	// export model to a text file
	//cplex.exportModel( "model.lp" );
	// set parameters
	setCPLEXParameters();

	// solve model
	cout << "Calling CPLEX solve ..." << endl;
	cplex.solve();

//	for(u_int i=0; i<n; i++)
//	{
//		cout << "u[" << i << "] = " << cplex.getValue(u[i]) << endl;
//		//printf("x_%d;\n",x[i]);
//	}
//	for(u_int i=0; i<n; i++)
//	{
//		cout << "y[" << i << "] = " << cplex.getValue(y[i]) << endl;
//		//printf("x_%d;\n",x[i]);
//	}
//	for(u_int i=0; i<2*m; i++)
//	{
//		cout << "x[" << i << "] = " << cplex.getValue(x[i]) << endl;
//		//printf("x_%d;\n",x[i]);
//	}

	cout << "CPLEX finished." << endl << endl;
	cout << "CPLEX status: " << cplex.getStatus() << endl;
	cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	cout << "Objective value: " << cplex.getObjValue() << endl;
	cout << "CPU time: " << Tools::CPUtime() << endl;
	cout << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;

	file_dat << "CPLEX finished." << endl << endl;
	file_dat << "CPLEX status: " << cplex.getStatus() << endl;
	file_dat << "Branch-and-Bound nodes: " << cplex.getNnodes() << endl;
	file_dat << "Objective value: " << cplex.getObjValue() << endl;
	file_dat << "CPU time: " << Tools::CPUtime() << endl;
	file_dat << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << endl << endl;
	file_dat.close();
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}
