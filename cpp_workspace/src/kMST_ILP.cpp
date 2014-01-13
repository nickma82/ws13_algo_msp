#include "kMST_ILP.h"
#include <sstream>      // std::ostringstream

kMST_ILP::kMST_ILP( Instance& _instance, string _model_type, int _k ) :
	instance( _instance ), model_type( _model_type ), k( _k )
{
	this->n = instance.n_nodes;
	this->m_edges = m = instance.n_edges;
	if( k == 0 ) k = n;

	this->outputFile.open("cplex_output", ios::out);
}

kMST_ILP::~kMST_ILP()
{
	// free global CPLEX resources
	this->cplex.end();
	this->model.end();
	this->env.end();

	this->outputFile.close();
}

void kMST_ILP::solve()
{
	try {
		// initialize CPLEX
		env = IloEnv();
		model = IloModel( env );

		// add model-specific constraints
		if( this->model_type == "scf" ) modelSCF();
		else if( this->model_type == "mcf" ) modelMCF();
		else if( this->model_type == "mtz" ) modelMTZ();
		else {
			std::cerr << "No existing model chosen\n";
			exit( -1 );
		}

		// build model
		cplex = IloCplex( model );
		// export model to a text file
		//cplex.exportModel( "model.lp" );
		// set parameters
		setCPLEXParameters();

		// solve model
		std::cout << "Calling CPLEX solve ...\n";
		cplex.solve();
		std::cout << "CPLEX finished." << std::endl;
		std::cout << "CPLEX status: " << cplex.getStatus() << std::endl;
		std::cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << std::endl;
		std::cout << "Objective value: " << cplex.getObjValue() << std::endl;
		std::cout << "CPU time: " << Tools::CPUtime() << std::endl;

		this->outputFile << "test1234" << std::endl;
	}
	catch( IloException& e ) {
		std::cerr << "kMST_ILP: exception " << e << std::endl;
		exit( -1 );
	}
	catch( ... ) {
		std::cerr << "kMST_ILP: unknown exception.\n";
		exit( -1 );
	}
}

// ----- private methods -----------------------------------------------

void kMST_ILP::setCPLEXParameters()
{
	// print every x-th line of node-log and give more details
	cplex.setParam( IloCplex::MIPInterval, 1 );
	cplex.setParam( IloCplex::MIPDisplay, 2 );
	// only use a single thread
	cplex.setParam( IloCplex::Threads, 1 );
}

void kMST_ILP::modelSCF() {
	/* Initialize the used edges */
	IloBoolVarArray x(this->env, this->m_edges * 2);
	std::ostringstream varName;

	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;
		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	/* 1 Create the objective function */
	IloExpr expression( this->env );
	for( size_t edgeNum = this->n-1; edgeNum < this->m_edges; ++edgeNum) {
		int edgeWeight = this->instance.edges[edgeNum].weight;

		expression += edgeWeight * x[edgeNum];
		expression += edgeWeight * x[edgeNum + this->m];
	}
	this->model.add( IloMinimize(this->env, expression) );
	expression.end();

	/* 11 Initialization of flow variable */
	IloIntVarArray flow( this->env, this->m_edges * 2 );
	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "flow_" <<  edgeNum;
		flow[edgeNum] = IloIntVar( this->env, 0, this->k, varName.str().c_str() );
	}

	/* 13 Initialization of y which is the support array
	 * for a k+1 node inclusion */
	IloBoolVarArray y( this->env, this->m_edges * 2 );
	for( size_t nodeNum = 0; nodeNum < this->m_edges *2; ++nodeNum ) {
		varName.str(""); varName.clear();
		varName << "y_" <<  nodeNum;
		y[nodeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// 7,8 if an edge x_ij is selected -> y_i and y_j must be 1
	for( size_t edgeNum = 0; edgeNum < this->m_edges * 2; ++edgeNum)
	{
		IloNumExpr Expr40( this->env ), Expr41( this->env );
		Expr40 += x[edgeNum];
		model.add( Expr40 <= y[instance.edges[ edgeNum % this->m_edges ].v1] );
		Expr40.end();

		Expr41 += x[edgeNum];
		model.add( Expr41 <= y[instance.edges[ edgeNum % this->m_edges ].v2] );
		Expr41.end();
	}

	// 10 exactly k+1 different nodes allowed (with artificial node)
	IloNumExpr Expr34( this->env );
	for(size_t v = 0; v < n; ++v )
	{
		Expr34 += y[v];
	}
	model.add(Expr34 == k+1);
	Expr34.end();

	// 9 @todo description //
	for(size_t edge = 0; edge < this->m_edges; ++edge )
	{
		IloNumExpr Expr50( this->env );
		Expr50 += y[instance.edges[edge].v1];
		Expr50 += x[edge];
		Expr50 += x[edge + this->m_edges];

		model.add(Expr50 <= y[instance.edges[edge].v2] + 1);
		Expr50.end();

		IloNumExpr Expr51( this->env );
		Expr51 += y[ instance.edges[edge].v2 ];
		Expr51 += x[ edge ];
		Expr51 += x[ edge + this->m_edges ];

		model.add(Expr51 <= y[instance.edges[edge].v1] + 1);
		Expr51.end();
	}


	// (2) // Flow expression from node 0 (flow k outgoing)
	IloNumExpr Expr2( this->env );
	for( auto it=instance.incidentEdges[0].begin();
			it != instance.incidentEdges.at(0).end(); ++it ) {
		if(instance.edges.at(*it).v1 == 0)
		{	// outgoing edge
			Expr2 += flow[*it];
			Expr2 -= flow[(*it)+m];
		}
		else
		{	// incoming edge
			Expr2 -= flow[*it];
			Expr2 += flow[(*it)+m];
		}

	}
	model.add(Expr2 == k);
	Expr2.end();


	// (4) // check flow
	for(size_t v = 1; v < this->n; ++v )
	{
		IloNumExpr Expr3( this->env );
		IloNumExpr Expr3_right( this->env );

		for( auto it = instance.incidentEdges[v].begin();
				it != instance.incidentEdges.at(v).end(); ++it )
		{
			if(instance.edges.at(*it).v1 == v)
			{	// outgoing edge
				Expr3 -= flow[*it];
				Expr3 += flow[(*it)+m];

				Expr3_right += flow[(*it)+m];
			}
			else
			{	// incoming edge
				Expr3 += flow[*it];
				Expr3 -= flow[(*it)+m];

				Expr3_right += flow[*it];
			}
		}
		model.add(Expr3 == IloMin(1,Expr3_right));

		Expr3.end();
		Expr3_right.end();
	}

	// (5) // force x[e] to be set if flow exist
	for(size_t e=0; e < this->m_edges * 2; ++e )
	{
		IloNumExpr Expr4( this->env );
		Expr4 += flow[e];
		model.add(Expr4 <= (k * x[e]));
		Expr4.end();
	}

	// (3) //
	IloNumExpr Expr5( this->env );
	for(size_t e=n-1; e < m; ++e )
	{
		Expr5 += x[e];
		Expr5 += x[e+m];
	}
	model.add(Expr5 == k-1);
	Expr5.end();

	// (6) // force the flow from node 0 to flow over exactly one edge
	IloNumExpr Expr6( this->env );
	for(size_t e=0; e < n-1; ++e )
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
	std::cout << "Calling CPLEX solve ..." << std::endl;
	cplex.solve();
	std::cout << "CPLEX finished." << std::endl;
	std::cout << "CPLEX status: " << cplex.getStatus() << std::endl;
	std::cout << "Branch-and-Bound nodes: " << cplex.getNnodes() << std::endl;
	std::cout << "Objective value: " << cplex.getObjValue() << std::endl;
	std::cout << "CPU time: " << Tools::CPUtime() << std::endl;
	std::cout << "Epsilon: " << cplex.getParam(IloCplex::EpInt) << std::endl;

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
