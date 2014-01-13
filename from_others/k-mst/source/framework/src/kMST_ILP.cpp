#include "kMST_ILP.h"

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
		cout << "CPU time: " << Tools::CPUtime() << endl << endl;
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
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build single commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
}

void kMST_ILP::modelMCF()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build multi commodity flow model
	// ++++++++++++++++++++++++++++++++++++++++++
}

void kMST_ILP::modelMTZ()
{
	// ++++++++++++++++++++++++++++++++++++++++++
	// TODO build Miller-Tucker-Zemlin model
	// ++++++++++++++++++++++++++++++++++++++++++
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	cplex.end();
	model.end();
	env.end();
}
