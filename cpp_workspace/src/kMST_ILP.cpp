#include "kMST_ILP.h"

kMST_Solution::kMST_Solution(std::string _testInstance, const int _k) :
	testInstance( _testInstance ), k( _k ) {
}

std::string kMST_Solution::getResultStream() {
	std::ostringstream outt;

	outt << "CPLEX status: " << this->cplexStatus << std::endl;
	outt << "Branch-and-Bound nodes: " << this->branchAndBoundNodes << std::endl;
	outt << "Objective value: " << this->objectiveValue << std::endl;
	outt << "CPU time: " << this->cpuTime << std::endl;

	return outt.str();
}

std::string kMST_Solution::getCompareableResultStream() {
	std::ostringstream outt;

	outt << this->testInstance << "\t\t& ";
	outt << this->k << "\t& ";
	outt << this->objectiveValue << "\t& ";
	outt << this->cpuTime << "\t& ";
	outt << this->branchAndBoundNodes << "\t";
	outt << "\\\\ \n";

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

kMST_Solution kMST_ILP::solve()
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
		solution.objectiveValue = cplex.getObjValue();
		solution.cpuTime = Tools::CPUtime();

		std::cout << solution.getResultStream();
		this->outputFile << solution.getCompareableResultStream();

		return solution;
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
	// (2) Initialize the decision variable
	//     We double the amount of edges to create a directed graph.
	//     0... edge not selected, 1... edge selected
	IloBoolVarArray x( this->env, this->m_edges * 2 );

	std::ostringstream varName;
	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;
		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (1) Create the objective function
	//     Starting by n-1 because the first n edges from node 0 to every other nodes and vice versa are ignored
	IloExpr expression1( this->env );
	for( size_t edgeNum = this->n-1; edgeNum < this->m_edges; ++edgeNum) {
		int edgeWeight = this->instance.edges[edgeNum].weight;

		expression1 += edgeWeight * x[edgeNum];
		expression1 += edgeWeight * x[edgeNum + this->m];
	}
	this->model.add( IloMinimize(this->env, expression1) );
	expression1.end();

	// (3) Initialize the new variable y
	//     0... node not selected, 1... node selected
	IloBoolVarArray y( this->env, this->m_edges * 2 );
	for( size_t nodeNum = 0; nodeNum < this->m_edges *2; ++nodeNum ) {
		varName.str(""); varName.clear();
		varName << "y_" <<  nodeNum;
		y[nodeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (4) Initialization of the flow variable
	//     0 <= f <= k
	IloIntVarArray flow( this->env, this->m_edges * 2 );
	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "flow_" <<  edgeNum;
		flow[edgeNum] = IloIntVar( this->env, 0, this->k, varName.str().c_str() );
	}

	// (5) The sum of all outgoing edges of node '0' must be k
	// (6) The sum of all incoming edges of node '0' must be 0
	IloNumExpr expression2 = IloNumExpr( this-> env );
	IloNumExpr expression3 = IloNumExpr( this-> env );
	for( auto it = instance.incidentEdges[0].begin();
			it != instance.incidentEdges[0].end(); ++it ) {
		if( instance.edges[*it].v1 == 0 ) {
			expression2 += flow[ *it ];
			expression3 += flow[ (*it) + this->m_edges ];
		}

	}
	this->model.add( expression2 == k );
	expression2.end();
	this->model.add( expression3 == 0 );
	expression3.end();

	// (7) We ensure that node '0' has exactly one edge selected
	IloNumExpr expression4( this->env );
	for(size_t e=0; e < n-1; ++e )
	{
		expression4 += x[e];
	}
	this->model.add(expression4 == 1);
	expression4.end();

	// (8) The sum of all selected edges must be k-1
	//     All edges from and to node '0' are ignored
	IloNumExpr expression5( this->env );
	for(size_t e = n-1; e < this->m_edges; ++e )
	{
		expression5 += x[e];
		expression5 += x[e + this->m_edges];
	}
	this->model.add(expression5 == k-1);
	expression5.end();

	// (9) For every node the sum of all incoming edges minus the sum of all outgoing edges must be 1 or 0
	for(size_t node = 1; node < this->n; ++node )
	{
		IloNumExpr expression6( this->env );
		IloNumExpr expression7( this->env );

		for( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it ) {
			if( instance.edges[*it].v1 == node ) {
				expression6 -= flow[ *it ];
				expression6 += flow[ (*it) + this->m_edges ];

				expression7 += flow[ (*it) + this->m_edges ];
			} else {
				expression6 += flow[ *it ];
				expression6 -= flow[ (*it) + this->m_edges ];

				expression7 += flow[ *it ];
			}
		}
		this->model.add( expression6 == IloMin( 1, expression7 ) );

		expression6.end();
		expression7.end();
	}

	// (10) If we have flow on an edge then the edge must be selected
	for(size_t edge = 0; edge < this->m_edges * 2; ++edge )
	{
		IloNumExpr expression8( this->env );
		expression8 += flow[edge];
		this->model.add(expression8 <= (k * x[edge]));
		expression8.end();
	}

	// (11) (12) Ensures that if an edge is selected, both nodes connected by the edge must be selected too
	for( size_t edgeNum = 0; edgeNum < this->m_edges * 2; ++edgeNum)
	{
		IloNumExpr expression9( this->env ), expression10( this->env );
		expression9 += x[edgeNum];
		this->model.add( expression9 <= y[instance.edges[ edgeNum % this->m_edges ].v1] );
		expression9.end();

		expression10 += x[edgeNum];
		this->model.add( expression10 <= y[instance.edges[ edgeNum % this->m_edges ].v2] );
		expression10.end();
	}

	// (13) Guarantees that only one edge connecting two nodes can be selected at the same time
	for(size_t edge = 0; edge < this->m_edges; ++edge )
	{
		IloNumExpr expression11( this->env );
		expression11 += y[ instance.edges[edge].v1 ];
		expression11 += x[ edge ];
		expression11 += x[ edge + this->m_edges ];

		this->model.add( expression11 <= y[ instance.edges[edge].v2 ] + 1);
		expression11.end();

		IloNumExpr expression12( this->env );
		expression12 += y[ instance.edges[edge].v2 ];
		expression12 += x[ edge ];
		expression12 += x[ edge + this->m_edges ];

		this->model.add( expression12 <= y[ instance.edges[edge].v1 ] + 1);
		expression12.end();
	}

	// (14) We assure that there are exactly k+1 (including the artifical node) nodes selected
	IloNumExpr expression13( this->env );
	for(size_t v = 0; v < n; ++v )
	{
		expression13 += y[v];
	}
	this->model.add(expression13 == k+1);
	expression13.end();

	// (15) Every node must have one or none incoming edge to avoid cycles
	for (size_t node = 1; node < n; ++node) {
		IloNumExpr expression14( this->env );

		for ( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it) {
			if (instance.edges[*it].v2 == node) {
				// incoming edge
				expression14 += x[ *it ];
			} else {
				// outgoing edge
				expression14 += x[ (*it) + this->m_edges ];
			}
		}
		model.add( expression14 <= 1 );
		expression14.end();
	}
}

void kMST_ILP::modelMCF() {
	// (17) Initialize the decision variable
	//      We double the amount of edges to create a directed graph.
	//      0... edge not selected, 1... edge selected
	IloBoolVarArray x( this->env, 2 * this->m_edges );
	std::ostringstream varName;
	for (size_t edgeNum = 0; edgeNum < 2 * this->m_edges ; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;

		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str());
	}

	// (16) Create the objective function
	//      Starting by n-1 because the first n edges from node 0 to every other nodes and vice versa are ignored
	IloExpr expression1( this->env);
	for (size_t e = n - 1; e < this->m_edges ; ++e ) {
		expression1 += instance.edges[e].weight * x[e];
		expression1 += instance.edges[e].weight * x[e + m];
	}
	model.add(IloMinimize( this->env, expression1));
	expression1.end();

	// (18) Initialize the new variable y
	//      0... node not selected, 1... node selected
	IloBoolVarArray y( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "y_" << node;

		y[node] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (19) Initialization of the flow variable
	//      There are |V| * |E| different flows
	//      0 <= f_{i,j}^n <= 1
	IloNumVarArray f( this->env, 2 * this->m_edges * (n - 1));
	for (size_t l = 0; l < n - 1; ++l ) {
		for (size_t e = 0; e < 2 * this->m_edges ; ++e ) {
			varName.str(""); varName.clear();
			varName << "f_" << e << "_" << l;
			f[(2 * this->m_edges * l) + e] = IloNumVar( this->env, 0, 1, varName.str().c_str());
		}
	}

	// (20) The artificial node '0' sends out k different flows of value 1 and there the sum of all outgoing flows must be k
	for (size_t l = 0; l < n - 1; ++l ) {
		IloNumExpr expression2( this->env);
		for (auto it = instance.incidentEdges[0].begin();
				it != instance.incidentEdges[0].end(); ++it ) {
			if (instance.edges.at(*it).v1 == 0) {	// outgoing edge
				expression2 += f[(2 * this->m_edges * l) + (*it)];
				expression2 -= f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
		model.add(expression2 <= 1);
		expression2.end();
	}

	// (21) The artificial node '0' sends out exactly k flows to k other nodes
	IloNumExpr expression3( this->env);
	for (size_t l = 0; l < n - 1; ++l ) {
		for (auto it = instance.incidentEdges[0].begin();
				it != instance.incidentEdges[0].end(); ++it ) {
			if (instance.edges.at(*it).v1 == 0) {	// outgoing edge
				expression3 += f[(2 * this->m_edges * l) + (*it)];
				expression3 -= f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
	}
	model.add( expression3 == k );
	expression3.end();

	// (22) All flows incoming in the target node must have a flow with a value with 0 or 1
	for (size_t l = 0; l < n - 1; ++l ) {
		IloNumExpr expression4( this->env);
		for (auto it = instance.incidentEdges.at(l + 1).begin();
				it != instance.incidentEdges.at(l + 1).end(); ++it ) {
			if (instance.edges.at(*it).v2 == l + 1) {	// incoming edge normal
				expression4 += f[(2 * this->m_edges * l) + (*it)];
			} else {	// incoming artificial edge
				expression4 += f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
		model.add(expression4 <= 1);
		expression4.end();
	}

	// (23) The sum of all incoming flows on their target node must be k
	IloNumExpr expression5( this->env);
	for (size_t l = 0; l < n - 1; ++l ) {
		for (auto it = instance.incidentEdges.at(l + 1).begin();
				it != instance.incidentEdges.at(l + 1).end(); ++it ) {
			if (instance.edges.at(*it).v2 == l + 1) {	// incoming edge normal
				expression5 += f[(2 * this->m_edges * l) + (*it)];
			} else {	// incoming artificial edge
				expression5 += f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
	}
	model.add(expression5 == k);
	expression5.end();

	// (24) The sum of all incoming flows minus the sum of all outgoing flows of each node, where the flow has
	//      another target, must be 0
	for (size_t l = 0; l < n - 1; ++l ) {
		for (size_t j = 1; j < n; ++j ) {
			if (j == l + 1)
				continue;

			IloNumExpr expression6( this->env);

			for (auto it = instance.incidentEdges[j].begin();
					it != instance.incidentEdges[j].end(); ++it ) {
				if (instance.edges.at(*it).v2 == j) {

					expression6 += f[(2 * this->m_edges * l) + (*it)];
					expression6 -= f[(2 * this->m_edges * l) + (*it + this->m_edges )];

				} else if ((instance.edges.at(*it).v1 == j)) {

					expression6 -= f[(2 * this->m_edges * l) + (*it)];
					expression6 += f[(2 * this->m_edges * l) + (*it + this->m_edges )];

				}
			}

			model.add(expression6 == 0);
			expression6.end();
		}
	}

	// (25) If a flow in on an edge, then the edge must be selected
	for (size_t l = 0; l < n - 1; ++l ) {
		for (size_t e = 0; e < 2 * this->m_edges ; ++e ) {
			IloNumExpr expression7( this->env);
			expression7 += f[(2 * l * this->m_edges ) + e];
			model.add(expression7 <= x[e]);
			expression7.end();
		}
	}

	// (26) The artificial node '0' has exactly one outgoing edge selected
	IloIntExpr expression8( this->env);
	for (size_t e = 0; e < n - 1; ++e ) {
		expression8 += x[e];
	}
	model.add(expression8 == 1);
	expression8.end();

	// (27) The sum of all edges selected (excluding the artificial node) must be k-1
	IloNumExpr expression9( this->env);
	for (size_t e = n - 1; e < this->m_edges ; ++e ) {
		expression9 += x[e];
		expression9 += x[e + m];
	}
	model.add(expression9 == k - 1);
	expression9.end();

	// (28) (29) Ensures that if an edge is selected, both nodes connected by the edge must be selected too
	for (size_t edge = 0; edge < this->m_edges * 2 ; ++edge ) {
		IloNumExpr expression10( this->env);
		expression10 += x[edge];
		model.add(expression10 <= y[instance.edges.at(edge % this->m_edges ).v1]);
		expression10.end();

		IloNumExpr expression11( this->env);
		expression11 += x[edge];
		model.add(expression11 <= y[instance.edges.at(edge % this->m_edges ).v2]);
		expression11.end();
	}

	// (30) Guarantees that only one edge connecting two nodes can be selected at the same time
	for (size_t e = 0; e < this->m_edges ; ++e ) {
		IloNumExpr expression12( this->env);
		expression12 += y[instance.edges[e].v1];
		expression12 += x[e];
		expression12 += x[e + m];

		model.add(expression12 <= y[instance.edges[e].v2] + 1);
		expression12.end();

		IloNumExpr expression13( this->env);
		expression13 += y[instance.edges[e].v2];
		expression13 += x[e];
		expression13 += x[e + m];

		model.add(expression13 <= y[instance.edges[e].v1] + 1);
		expression13.end();
	}

	// (31) We assure that there are exactly k+1 (including the artifical node) nodes selected
	IloNumExpr expression14( this->env);
	for (size_t v = 0; v < n; ++v ) {
		expression14 += y[v];
	}
	model.add(expression14 == k + 1);
	expression14.end();

	// (32) Every node must have one or none incoming edge to avoid cycles
	for (size_t node = 1; node < n; ++node) {
		IloNumExpr expression15( this->env );

		for ( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it) {
			if (instance.edges[*it].v2 == node) {
				// incoming edge
				expression15 += x[ *it ];
			} else {
				// outgoing edge
				expression15 += x[ (*it) + this->m_edges ];
			}
		}
		model.add( expression15 <= 1 );
		expression15.end();
	}
}

void kMST_ILP::modelMTZ() {
	// (34) Initialize the decision variable
	//      We double the amount of edges to create a directed graph.
	//      0... edge not selected, 1... edge selected
	IloBoolVarArray x( this->env, 2 * this->m_edges );
	std::ostringstream varName;
	for (size_t edgeNum = 0; edgeNum < 2 * this->m_edges ; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;

		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str());
	}

	// (33) Create the objective function
	//      Starting by n-1 because the first n edges from node 0 to every other nodes and vice versa are ignored
	IloExpr expression1( this->env);
	for (size_t e = n - 1; e < this->m_edges ; ++e ) {
		expression1 += instance.edges[e].weight * x[e];
		expression1 += instance.edges[e].weight * x[e + m];
	}
	model.add(IloMinimize( this->env, expression1));
	expression1.end();

	// (35) Initialize the new variable y
	//      0... node not selected, 1... node selected
	IloBoolVarArray y( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "y_" << node;

		y[node] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (36) Initialization of order variable u
	//	Each node has an order with a value between 0 and k
	//	0 <= u <= k
	IloIntVarArray u( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "u_" << node;
		u[ node ] = IloIntVar(this->env, 0, k, varName.str().c_str());
	}

	// (37) The order of the artificial node '0' must be 0
	IloNumExpr expression2( this->env );
	expression2 += u[0];
	model.add(expression2 == 0);
	expression2.end();

	// (38) Exactly one outgoing edge from the artificial node must be selected
	IloNumExpr expression3( this->env );
	for (size_t e = 0; e < n - 1; ++e) {
		expression3 += x[e];
	}
	model.add(expression3 == 1);
	expression3.end();

	// (39) Ensures that an edge can only be selected between a node with a lower and a target node with a higher order
	for (size_t edge = 0; edge < this->m_edges; ++edge) {
		IloNumExpr expression4( this->env );
		expression4 += u[instance.edges[edge].v1];
		expression4 += x[edge];

		model.add(expression4 <= u[instance.edges[edge].v2] + k * (1 - x[edge]));
		expression4.end();

		IloNumExpr expression5( this->env );
		expression5 += u[instance.edges[edge].v2];
		expression5 += x[(edge) + this->m_edges];

		model.add(expression5 <= u[instance.edges[edge].v1] + k * (1 - x[(edge) + this->m_edges]));
		expression5.end();
	}

	// (40) The sum over all order variables u must be (k*(k+1)) / 2 
	//	Here we guarantee that no order variable is left out
	IloNumExpr expression6( this->env );
	for (size_t v = 0; v < n; ++v) {
		expression6 += u[v];
	}
	model.add(expression6 == (k * (k + 1)) / 2);
	expression6.end();

	// (41) We ensure that if a node has an order, then the node must also be selected
	for (size_t v = 0; v < n; ++v) {
		IloNumExpr expression7( this->env );
		expression7 += u[v];
		model.add(expression7 <= k * y[v]);
		expression7.end();
	}

	// (42) The sum of all edges selected (excluding the artificial node) must be k-1
	IloNumExpr expression8( this->env );
	for (size_t e = n - 1; e < this->m_edges; ++e) {
		expression8 += x[e];
		expression8 += x[e + this->m_edges];
	}
	model.add(expression8 == k - 1);
	expression8.end();

	// (43) We assure that there are exactly k (excluding the artifical node) nodes selected
	//	The artificial node must not have been selected because of (39)
	IloNumExpr expression9( this->env );
	for (size_t v = 1; v < n; ++v) {
		expression9 += y[v];
	}
	model.add(expression9 == k);
	expression9.end();

	// (44) Every node must have one or none incoming edge to avoid cycles
	for (size_t node = 1; node < n; ++node) {
		IloNumExpr expression10( this->env );

		for ( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it) {
			if (instance.edges[*it].v2 == node) {
				// incoming edge
				expression10 += x[ *it ];
			} else {
				// outgoing edge
				expression10 += x[ (*it) + this->m_edges ];
			}
		}
		model.add( expression10 <= 1 );
		expression10.end();
	}

	// (45) (46) A node can only have an outgoing or incoming edge selected if the order of the node is greater then 0
	//	     Excluding the artificial node
	for (size_t v = 1; v < n; ++v) {
		for ( auto it = instance.incidentEdges[v].begin();
				it != instance.incidentEdges[v].end(); ++it) {
			if ( instance.edges[*it].v1 == 0 ) {
				IloNumExpr expression11( this->env );
				IloNumExpr expression12( this->env );
				expression11 += x[(*it)];
				expression12 += x[(*it) + this->m_edges];
				model.add(expression11 <= u[instance.edges[*it].v2]);
				model.add(expression12 <= u[instance.edges[*it].v2]);
				expression11.end();
				expression12.end();
			} else {
				IloNumExpr expression13( this->env );
				IloNumExpr expression14( this->env );
				expression13 += x[(*it)];
				expression14 += x[(*it)];
				model.add(expression13 <= u[instance.edges[*it].v1]);
				model.add(expression14 <= u[instance.edges[*it].v2]);
				expression13.end();
				expression14.end();

				IloNumExpr expression15( this->env );
				IloNumExpr expression16( this->env );
				expression15 += x[(*it) + this->m_edges];
				expression16 += x[(*it) + this->m_edges];
				model.add(expression15 <= u[instance.edges[*it].v1]);
				model.add(expression16 <= u[instance.edges[*it].v2]);
				expression15.end();
				expression16.end();
			}
		}
	}

	// (47) Ensures that between two selected nodes only one selected edge can exist
	for (size_t edge = n - 1; edge < this->m_edges; ++edge) {
		IloNumExpr expression17( this->env );
		expression17 += y[ instance.edges[edge].v1 ];
		expression17 += x[edge];
		expression17 += x[edge + this->m_edges];

		model.add(expression17 <= y[ instance.edges[edge].v2 ] + 1);
		expression17.end();

		IloNumExpr expression18( this->env );
		expression18 += y[ instance.edges[edge].v2 ];
		expression18 += x[ edge ];
		expression18 += x[ edge + this->m_edges ];

		model.add(expression18 <= y[ instance.edges[edge].v1 ] + 1);
		expression18.end();
	}
}
