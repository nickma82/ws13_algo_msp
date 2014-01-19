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
	cplex.setParam( IloCplex::Threads, 4 );
}

void kMST_ILP::modelSCF() {
	// (2) Initialize the decision variable
	//     We double the amount of edges to create a directed graph.
	//     0... edge not selected, 1... edge selected
	IloNumExpr NumExpr1, NumExpr2;
	IloBoolVarArray x( this->env, this->m_edges * 2 );

	std::ostringstream varName;
	for( size_t edgeNum = 0; edgeNum < this->m_edges *2; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;
		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (1) Create the objective function
	//     Starting by n-1 because the first n edges from node 0 to every other nodes and vice versa are ignored
	IloExpr expression( this->env );
	for( size_t edgeNum = this->n-1; edgeNum < this->m_edges; ++edgeNum) {
		int edgeWeight = this->instance.edges[edgeNum].weight;

		expression += edgeWeight * x[edgeNum];
		expression += edgeWeight * x[edgeNum + this->m];
	}
	this->model.add( IloMinimize(this->env, expression) );
	expression.end();

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
	NumExpr1 = IloNumExpr( this-> env );
	NumExpr2 = IloNumExpr( this-> env );
	for( auto it = instance.incidentEdges[0].begin();
			it != instance.incidentEdges[0].end(); ++it ) {
		if( instance.edges[*it].v1 == 0 ) {
			NumExpr1 += flow[ *it ];
			NumExpr2 += flow[ (*it) + this->m_edges ];
		}

	}
	this->model.add( NumExpr1 == k );
	NumExpr1.end();
	this->model.add( NumExpr2 == 0 );
	NumExpr2.end();

	// (7) We ensure that node '0' has exactly one edge selected
	IloNumExpr Expr6( this->env );
	for(size_t e=0; e < n-1; ++e )
	{
		Expr6 += x[e];
	}
	this->model.add(Expr6 == 1);
	Expr6.end();

	// (8) The sum of all selected edges must be k-1
	//     All edges from and to node '0' are ignored
	IloNumExpr Expr5( this->env );
	for(size_t e = n-1; e < this->m_edges; ++e )
	{
		Expr5 += x[e];
		Expr5 += x[e + this->m_edges];
	}
	this->model.add(Expr5 == k-1);
	Expr5.end();

	// (9) For every node the sum of all incoming edges minus the sum of all outgoing edges must be 1 or 0
	for(size_t node = 1; node < this->n; ++node )
	{
		IloNumExpr Expr3( this->env );
		IloNumExpr Expr3_right( this->env );

		for( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it ) {
			if( instance.edges[*it].v1 == node ) {
				Expr3 -= flow[ *it ];
				Expr3 += flow[ (*it) + this->m_edges ];

				Expr3_right += flow[ (*it) + this->m_edges ];
			} else {
				Expr3 += flow[ *it ];
				Expr3 -= flow[ (*it) + this->m_edges ];

				Expr3_right += flow[ *it ];
			}
		}
		this->model.add( Expr3 == IloMin( 1, Expr3_right ) );

		Expr3.end();
		Expr3_right.end();
	}

	// (10) If we have flow on an edge then the edge must be selected
	for(size_t edge = 0; edge < this->m_edges * 2; ++edge )
	{
		IloNumExpr Expr4( this->env );
		Expr4 += flow[edge];
		this->model.add(Expr4 <= (k * x[edge]));
		Expr4.end();
	}

	// (11) (12) Ensures that if an edge is selected, both nodes connected by the edge must be selected too
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

	// (13) Guarantees that only one edge connecting two nodes can be selected at the same time
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

	// (14) We assure that there are exactly k+1 (including the artifical node) nodes selected
	IloNumExpr Expr34( this->env );
	for(size_t v = 0; v < n; ++v )
	{
		Expr34 += y[v];
	}
	this->model.add(Expr34 == k+1);
	Expr34.end();
}

void kMST_ILP::modelMCF() {
	// (16) Initialize the decision variable
	//      We double the amount of edges to create a directed graph.
	//      0... edge not selected, 1... edge selected
	IloBoolVarArray x( this->env, 2 * this->m_edges );
	std::ostringstream varName;
	for (size_t edgeNum = 0; edgeNum < 2 * this->m_edges ; ++edgeNum ) {
		varName.str(""); varName.clear();
		varName << "edge_" <<  edgeNum;

		x[edgeNum] = IloBoolVar( this->env, varName.str().c_str());
	}

	// (15) Create the objective function
	//      Starting by n-1 because the first n edges from node 0 to every other nodes and vice versa are ignored
	IloExpr expr( this->env);
	for (size_t e = n - 1; e < this->m_edges ; ++e ) {
		expr += instance.edges[e].weight * x[e];
		expr += instance.edges[e].weight * x[e + m];
	}
	model.add(IloMinimize( this->env, expr));
	expr.end();

	// (17) Initialize the new variable y
	//      0... node not selected, 1... node selected
	IloBoolVarArray y( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "y_" << node;

		y[node] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// (18) Initialization of the flow variable
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

	// (20) TODO
	for (size_t l = 0; l < n - 1; ++l ) {
		IloNumExpr Expr_left( this->env);
		for (auto it = instance.incidentEdges[0].begin();
				it != instance.incidentEdges[0].end(); ++it ) {
			if (instance.edges.at(*it).v1 == 0) {	// outgoing edge
				Expr_left += f[(2 * this->m_edges * l) + (*it)];
				Expr_left -= f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}

		IloNumExpr Expr_right( this->env);
		for (auto it = instance.incidentEdges.at(l + 1).begin();
				it != instance.incidentEdges.at(l + 1).end(); ++it ) {
			if (instance.edges.at(*it).v2 == l + 1) {	// incoming edge normal
				Expr_right += f[(2 * this->m_edges * l) + (*it)];
			} else {	// incoming artificial edge
				Expr_right += f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}

		model.add(Expr_left == Expr_right);
		Expr_left.end();
		Expr_right.end();
	}

	// (19) The artificial node '0' sends out TODO
	for (size_t l = 0; l < n - 1; ++l ) {
		IloNumExpr Expr2( this->env);
		for (auto it = instance.incidentEdges[0].begin();
				it != instance.incidentEdges[0].end(); ++it ) {
			if (instance.edges.at(*it).v1 == 0) {	// outgoing edge
				Expr2 += f[(2 * this->m_edges * l) + (*it)];
				Expr2 -= f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
		model.add(Expr2 <= 1);
		Expr2.end();
	}

	// (20) The artificial node '0' sends out exactly k flows to k other nodes
	IloNumExpr Expr3( this->env);
	for (size_t l = 0; l < n - 1; ++l ) {
		for (auto it = instance.incidentEdges[0].begin();
				it != instance.incidentEdges[0].end(); ++it ) {
			if (instance.edges.at(*it).v1 == 0) {	// outgoing edge
				Expr3 += f[(2 * this->m_edges * l) + (*it)];
				Expr3 -= f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
	}
	model.add( Expr3 == k );
	Expr3.end();

	// (21) All flows incoming in the target node must have a flow with a value with 0 or 1
	for (size_t l = 0; l < n - 1; ++l ) {
		IloNumExpr Expr4( this->env);
		for (auto it = instance.incidentEdges.at(l + 1).begin();
				it != instance.incidentEdges.at(l + 1).end(); ++it ) {
			if (instance.edges.at(*it).v2 == l + 1) {	// incoming edge normal
				Expr4 += f[(2 * this->m_edges * l) + (*it)];
			} else {	// incoming artificial edge
				Expr4 += f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
		model.add(Expr4 <= 1);
		Expr4.end();
	}

	// (22) The sum of all incoming flows on their target node must be k
	IloNumExpr Expr5( this->env);
	for (size_t l = 0; l < n - 1; ++l ) {
		for (auto it = instance.incidentEdges.at(l + 1).begin();
				it != instance.incidentEdges.at(l + 1).end(); ++it ) {
			if (instance.edges.at(*it).v2 == l + 1) {	// incoming edge normal
				Expr5 += f[(2 * this->m_edges * l) + (*it)];
			} else {	// incoming artificial edge
				Expr5 += f[(2 * this->m_edges * l) + ((*it) + this->m_edges )];
			}
		}
	}
	model.add(Expr5 == k);
	Expr5.end();

	// (23) The sum of all incoming flows minus the sum of all outgoing flows of each node, where the flow has
	//      another target, must be 0
	for (size_t l = 0; l < n - 1; ++l ) {
		for (size_t j = 1; j < n; ++j ) {
			if (j == l + 1)
				continue;

			IloNumExpr Expr6( this->env);

			for (auto it = instance.incidentEdges[j].begin();
					it != instance.incidentEdges[j].end(); ++it ) {
				if (instance.edges.at(*it).v2 == j) {

					Expr6 += f[(2 * this->m_edges * l) + (*it)];
					Expr6 -= f[(2 * this->m_edges * l) + (*it + this->m_edges )];

				} else if ((instance.edges.at(*it).v1 == j)) {

					Expr6 -= f[(2 * this->m_edges * l) + (*it)];
					Expr6 += f[(2 * this->m_edges * l) + (*it + this->m_edges )];

				}
			}

			model.add(Expr6 == 0);
			Expr6.end();
		}
	}

	// (24) If a flow in on an edge, then the edge must be selected
	for (size_t l = 0; l < n - 1; ++l ) {
		for (size_t e = 0; e < 2 * this->m_edges ; ++e ) {
			IloNumExpr Expr7( this->env);
			Expr7 += f[(2 * l * this->m_edges ) + e];
			model.add(Expr7 <= x[e]);
			Expr7.end();
		}
	}

	// (25) The artificial node '0' has exactly one outgoing edge selected
	IloIntExpr Expr9( this->env);
	for (size_t e = 0; e < n - 1; ++e ) {
		Expr9 += x[e];
	}
	model.add(Expr9 == 1);
	Expr9.end();

	// (26) The sum of all edges selected (ecluding the artificial node) must be k-1
	IloNumExpr Expr8( this->env);
	for (size_t e = n - 1; e < this->m_edges ; ++e ) {
		Expr8 += x[e];
		Expr8 += x[e + m];
	}
	model.add(Expr8 == k - 1);
	Expr8.end();

	// (27) (28) Ensures that if an edge is selected, both nodes connected by the edge must be selected too
	for (size_t edge = 0; edge < this->m_edges * 2 ; ++edge ) {
		IloNumExpr ExprNode1Selected( this->env);
		ExprNode1Selected += x[edge];
		model.add(ExprNode1Selected <= y[instance.edges.at(edge % this->m_edges ).v1]);
		ExprNode1Selected.end();

		IloNumExpr ExprNode2Selected( this->env);
		ExprNode2Selected += x[edge];
		model.add(ExprNode2Selected <= y[instance.edges.at(edge % this->m_edges ).v2]);
		ExprNode2Selected.end();
	}

	// (29) Guarantees that only one edge connecting two nodes can be selected at the same time
	for (size_t e = 0; e < this->m_edges ; ++e ) {
		IloNumExpr Expr50( this->env);
		Expr50 += y[instance.edges[e].v1];
		Expr50 += x[e];
		Expr50 += x[e + m];

		model.add(Expr50 <= y[instance.edges[e].v2] + 1);
		Expr50.end();

		IloNumExpr Expr51( this->env);
		Expr51 += y[instance.edges[e].v2];
		Expr51 += x[e];
		Expr51 += x[e + m];

		model.add(Expr51 <= y[instance.edges[e].v1] + 1);
		Expr51.end();
	}

	// (30) We assure that there are exactly k+1 (including the artifical node) nodes selected
	IloNumExpr Expr34( this->env);
	for (size_t v = 0; v < n; ++v ) {
		Expr34 += y[v];
	}
	model.add(Expr34 == k + 1);
	Expr34.end();
}

void kMST_ILP::modelMTZ() {
	std::ostringstream varName;

	// 42 Initialization of x (array of used edges in the solution)
	// x .. used for the edges
	IloBoolVarArray x( this->env, 2 * this->m_edges);
	for (size_t edge = 0; edge < 2 * this->m_edges; ++edge) {
		varName.str(""); varName.clear();
		varName << "x_" << edge;
		x[edge] = IloBoolVar(this->env, varName.str().c_str() );
	}

	// Initialization of u (array of used nodes in the solution)
	IloIntVarArray u( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "u_" << node;
		u[ node ] = IloIntVar(this->env, 0, k, varName.str().c_str());
	}

	// 43 Initialization of y (support array to include only k nodes)
	// Sum of y == k with y El. {0,1}
	IloBoolVarArray y( this->env, this->n);
	for (size_t node = 0; node < this->n; ++node) {
		varName.str(""); varName.clear();
		varName << "y_" << node;
		y[node] = IloBoolVar( this->env, varName.str().c_str() );
	}

	// 29 objective function
	IloExpr expr( this->env );
	for (size_t edge = this->n - 1; edge < this->m_edges; ++edge) {
		expr += instance.edges[edge].weight * x[edge];
		expr += instance.edges[edge].weight * x[edge + this->m_edges];
	}
	model.add(IloMinimize( this->env, expr));
	expr.end();

	/* adding constraints */
	// 31 u[v] has to be between 0 and k
	for (size_t v = 1; v < n; ++v) {
		IloNumExpr Expr2( this->env );
		Expr2 += u[v];
		model.add(Expr2 <= k);
		Expr2.end();
	}

	// 32 u[0] must be 0
	IloNumExpr Expr5( this->env );
	Expr5 += u[0];
	model.add(Expr5 == 0);
	Expr5.end();

	// 33 exactly 1 outgoing edge from the root
	IloNumExpr Expr4( this->env );
	for (size_t e = 0; e < n - 1; ++e) {
		Expr4 += x[e];
	}
	model.add(Expr4 == 1);
	Expr4.end();

	// 34 k-1 edges are allowed in the solution
	IloNumExpr Expr6( this->env );
	for (size_t e = n - 1; e < this->m_edges; ++e) {
		Expr6 += x[e];
		Expr6 += x[e + this->m_edges];
	}
	model.add(Expr6 == k - 1);
	Expr6.end();

	// 35 sum of the u value
	IloNumExpr Expr7( this->env );
	for (size_t v = 0; v < n; ++v) {
		Expr7 += u[v];
	}
	model.add(Expr7 == (k * (k + 1)) / 2);
	Expr7.end();

	// 36 force y to be 1 if u is in the solution
	for (size_t v = 0; v < n; ++v) {
		IloNumExpr Expr33( this->env );
		Expr33 += u[v];
		model.add(Expr33 <= k * y[v]);
		Expr33.end();
	}

	// 37 only k different nodes allowed
	IloNumExpr Expr34( this->env );
	for (size_t v = 0; v < n; ++v) {
		Expr34 += y[v];
	}
	model.add(Expr34 == k);
	Expr34.end();

	// (39) (40) // if f_ij in the solution -> u[i] and u[j] must be greater than 0
	for (size_t v = 1; v < n; ++v) {
		for ( auto it = instance.incidentEdges[v].begin();
				it != instance.incidentEdges[v].end(); ++it) {
			if ( instance.edges[*it].v1 == 0 ) {
				IloNumExpr Expr6( this->env );
				IloNumExpr Expr7( this->env );
				Expr6 += x[(*it)];
				Expr7 += x[(*it) + this->m_edges];
				model.add(Expr6 <= u[instance.edges[*it].v2]);
				model.add(Expr7 <= u[instance.edges[*it].v2]);
				Expr6.end();
				Expr7.end();
			} else {
				IloNumExpr Expr6( this->env );
				IloNumExpr Expr7( this->env );
				Expr6 += x[(*it)];
				Expr7 += x[(*it)];
				model.add(Expr6 <= u[instance.edges[*it].v1]);
				model.add(Expr7 <= u[instance.edges[*it].v2]);
				Expr6.end();
				Expr7.end();

				IloNumExpr Expr40( this->env );
				IloNumExpr Expr41( this->env );
				Expr40 += x[(*it) + this->m_edges];
				Expr41 += x[(*it) + this->m_edges];
				model.add(Expr40 <= u[instance.edges[*it].v1]);
				model.add(Expr41 <= u[instance.edges[*it].v2]);
				Expr40.end();
				Expr41.end();
			}
		}
	}

	// 30 miller tucker zemlin formulation
	for (size_t edge = 0; edge < this->m_edges; ++edge) {
		IloNumExpr Expr4( this->env );
		Expr4 += u[instance.edges[edge].v1];
		Expr4 += x[edge];

		model.add(Expr4 <= u[instance.edges[edge].v2] + k * (1 - x[edge]));
		Expr4.end();

		IloNumExpr Expr12( this->env );
		Expr12 += u[instance.edges[edge].v2];
		Expr12 += x[(edge) + this->m_edges];

		model.add(Expr12 <= u[instance.edges[edge].v1] + k * (1 - x[(edge) + this->m_edges]));
		Expr12.end();
	}

	// 41
	for (size_t edge = n - 1; edge < this->m_edges; ++edge) {
		IloNumExpr Expr50( this->env );
		Expr50 += y[ instance.edges[edge].v1 ];
		Expr50 += x[edge];
		Expr50 += x[edge + this->m_edges];

		model.add(Expr50 <= y[ instance.edges[edge].v2 ] + 1);
		Expr50.end();

		IloNumExpr Expr51( this->env );
		Expr51 += y[ instance.edges[edge].v2 ];
		Expr51 += x[ edge ];
		Expr51 += x[ edge + this->m_edges ];

		model.add(Expr51 <= y[ instance.edges[edge].v1 ] + 1);
		Expr51.end();
	}

	// 38 every node \ 0 must have an incoming flow of <= 1
	for (size_t node = 1; node < n; ++node) {
		IloNumExpr Expr11( this->env );

		for ( auto it = instance.incidentEdges[node].begin();
				it != instance.incidentEdges[node].end(); ++it) {
			if (instance.edges[*it].v2 == node) {
				// incoming edge
				Expr11 += x[ *it ];
			} else {
				// outgoing edge
				Expr11 += x[ (*it) + this->m_edges ];
			}
		}
		model.add( Expr11 <= 1 );
		Expr11.end();
	}
}
