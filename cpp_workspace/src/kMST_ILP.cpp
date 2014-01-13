#include "kMST_ILP.h"

kMST_Solution::kMST_Solution(std::string _testInstance, const int _k) :
	testInstance( _testInstance ), k( _k ) {
}

std::string kMST_Solution::getResultStream() {
	std::ostringstream outt;

	outt << "CPLEX status: " << this->cplexStatus << std::endl;
	outt << "Branch-and-Bound nodes: " << this->branchAndBoundNodes << std::endl;
	outt << "Objective value: " << this->objectValue << std::endl;
	outt << "CPU time: " << this->cpuTime << std::endl;

	return outt.str();
}

kMST_ILP::kMST_ILP( Instance& _instance, string _model_type, int _k ) :
	instance( _instance ), model_type( _model_type ), k( _k )
{
	this->n = instance.n_nodes;
	this->m_edges = m = instance.n_edges;
	if( k == 0 )
		k = n;

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

		/* Form the solution */
		std::ostringstream tmpStatus;
		tmpStatus << cplex.getStatus();

		kMST_Solution solution(this->instance.name, this->k);
		solution.cplexStatus = tmpStatus.str();
		solution.branchAndBoundNodes = cplex.getNnodes();
		solution.objectValue = cplex.getObjValue();
		solution.cpuTime = Tools::CPUtime();

		std::cout << solution.getResultStream();
		this->outputFile << solution.getResultStream();
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
	cplex.setParam( IloCplex::Threads, 4 );
}

void kMST_ILP::modelSCF(bool makeFasterResults) {
	/* Initialize the used edges */
	IloBoolVarArray x( this->env, this->m_edges * 2 );
	std::ostringstream varName;

	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;
		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// 1 Create the objective function
	// n-1 because edges beginning from 0 are ignored
	IloExpr expression( this->env );
	for( size_t edgeNum = this->n-1; edgeNum < this->m_edges; ++edgeNum) {
		int edgeWeight = this->instance.edges[edgeNum].weight;

		expression += edgeWeight * x[edgeNum];
		expression += edgeWeight * x[edgeNum + this->m];
	}
	this->model.add( IloMinimize(this->env, expression) );
	expression.end();

	// 11 Initialization of flow variable
	IloIntVarArray flow( this->env, this->m_edges * 2 );
	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "flow_" <<  edgeNum;
		flow[edgeNum] = IloIntVar( this->env, 0, this->k, varName.str().c_str() );
	}

	// flow from j to "0" is k
	IloNumExpr Expr2( this->env ), Expr3( this->env );
	for( auto it = instance.incidentEdges[0].begin();
			it != instance.incidentEdges[0].end(); ++it ) {
		if( instance.edges[*it].v1 == 0 ) {
			// outgoing edge
			Expr2 += flow[ *it ];
			Expr3 += flow[ (*it)+m ];
		}

	}
	this->model.add( Expr2 == k );
	Expr2.end();
	this->model.add( Expr3 == 0 ); //returning flow forbidden
	Expr3.end();


	// 4 check flow (for all nodes != 0)
	for(size_t node = 1; node < this->n; ++node )
	{
		IloNumExpr Expr3( this->env );
		IloNumExpr Expr3_right( this->env );

		for( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges.at(node).end(); ++it ) {
			if( instance.edges[*it].v1 == node ) {
				// outgoing edge
				Expr3 -= flow[ *it ];
				Expr3 += flow[ (*it) + m ];

				Expr3_right += flow[ (*it) + m ];
			} else {
				// incoming edge
				Expr3 += flow[ *it ];
				Expr3 -= flow[ (*it) + m ];

				Expr3_right += flow[ *it ];
			}
		}
		this->model.add( Expr3 == IloMin(1,Expr3_right) );

		Expr3.end();
		Expr3_right.end();
	}

	// 5 force x[e] to be set if flow exist
	for(size_t edge = 0; edge < this->m_edges * 2; ++edge )
	{
		IloNumExpr Expr4( this->env );
		Expr4 += flow[edge];
		this->model.add(Expr4 <= (k * x[edge]));
		Expr4.end();
	}

	// 3 @todo description
	IloNumExpr Expr5( this->env );
	for(size_t e=n-1; e < m; ++e )
	{
		Expr5 += x[e];
		Expr5 += x[e+m];
	}
	this->model.add(Expr5 == k-1);
	Expr5.end();

	// 6  force the flow from node 0 to flow over exactly one edge
	IloNumExpr Expr6( this->env );
	for(size_t e=0; e < n-1; ++e )
	{
		Expr6 += x[e];
	}
	this->model.add(Expr6 == 1);
	Expr6.end();

	if ( makeFasterResults ) {
		// 13 Initialization of y which is the support array
		// for a k+1 node inclusion
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
			this->model.add( Expr40 <= y[instance.edges[ edgeNum % this->m_edges ].v1] );
			Expr40.end();

			Expr41 += x[edgeNum];
			this->model.add( Expr41 <= y[instance.edges[ edgeNum % this->m_edges ].v2] );
			Expr41.end();
		}

		// 10 exactly k+1 different nodes allowed (with artificial node)
		IloNumExpr Expr34( this->env );
		for(size_t v = 0; v < n; ++v )
		{
			Expr34 += y[v];
		}
		this->model.add(Expr34 == k+1);
		Expr34.end();

		// 9 @todo description //
		for(size_t edge = 0; edge < this->m_edges; ++edge )
		{
			IloNumExpr Expr50( this->env );
			Expr50 += y[ instance.edges[edge].v1 ];
			Expr50 += x[ edge ];
			Expr50 += x[ edge + this->m_edges ];

			this->model.add( Expr50 <= y[ instance.edges[edge].v2 ] + 1);
			Expr50.end();

			IloNumExpr Expr51( this->env );
			Expr51 += y[ instance.edges[edge].v2 ];
			Expr51 += x[ edge ];
			Expr51 += x[ edge + this->m_edges ];

			this->model.add( Expr51 <= y[ instance.edges[edge].v1 ] + 1);
			Expr51.end();
		}
	} //end makeFasterResults
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
