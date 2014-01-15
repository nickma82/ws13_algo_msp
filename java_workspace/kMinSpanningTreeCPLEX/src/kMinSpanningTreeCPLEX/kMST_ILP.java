package kMinSpanningTreeCPLEX;

import ilog.concert.IloException;
import ilog.concert.IloIntExpr;
import ilog.concert.IloIntVar;
import ilog.concert.IloLinearIntExpr;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;

public class kMST_ILP {
	// input data
	private Instance instance;
	private String model_type;
	private int k;

	// number of edges and nodes including root node and root edges
	private int m, n;

	IloCplex cplex;
	private IloIntVar[] f;
	private IloIntVar[] x;

	public kMST_ILP(Instance instance, String model_type, int k) {
		this.instance = instance;
		this.model_type = model_type;
		this.k = k;

		this.n = instance.getNumberOfNodes();
		this.m = instance.getNumberOfEdges();
		if (this.k == 0) {
			this.k = this.n;
		}
	}

	public void solve() {
		try {
			cplex = new IloCplex();

			if (model_type.equals("scf")) {
				modelSCF();
			} else if (model_type.equals("mcf")) {
				modelMCF();
			} else if (model_type.equals("mtz")) {
				modelMTZ();
			} else {
				System.err.println("No existing model chosen");
				System.exit(-1);
			}

			// set parameters
			setCPLEXParameters();

			// export model to a text file
			// cplex.exportModel("model.lp");

			// solve model
			System.out.println("Calling CPLEX solve ...");
			cplex.solve();
			System.out.println("CPLEX finished.");
			System.out.println();
			System.out.println("CPLEX status: " + cplex.getStatus());
			System.out.println("Branch-and-Bound nodes: " + cplex.getNnodes());
			System.out.println("Objective value: " + cplex.getObjValue());
			System.out.println("CPU time: " + Tools.CPUtime());
			System.out.println();

			if (this.model_type.equals("scf")) {
				System.out.println("Variables:");
				for (int i = 0; i < 2 * m; i++) {
					double value = cplex.getValue(f[i]);
					double selected = cplex.getValue(x[i]);
					if (selected > 0) {
						System.out.println(i + " - " + instance.getEdge(i % m)
								+ ": " + selected + ", " + value);
					}
				}
			}

		} catch (IloException e) {
			System.err.println("kMST_ILP: exception " + e.getMessage());
			System.exit(-1);
		}
		cplex.end();
	}

	private void setCPLEXParameters() throws IloException {
		// print every x-th line of node-log and give more details
		cplex.setParam(IloCplex.IntParam.MIPInterval, 1);
		cplex.setParam(IloCplex.IntParam.MIPDisplay, 2);
		// only use a single thread
		cplex.setParam(IloCplex.IntParam.Threads, 1);
	}

	private void modelSCF() throws IloException {
		// (1) edge variables
		x = cplex.boolVarArray(2 * m);

		// (2) objective function
		IloLinearIntExpr obj = cplex.linearIntExpr();
		for (int e = 0; e < m; e++) {
			int weight = this.instance.getEdge(e).getWeight();
			obj.addTerm(x[e], weight);
			obj.addTerm(x[e + m], weight);
		}
		cplex.addMinimize(obj);

		// (3) flow variable
		String[] names = new String[2 * m];
		for (int i = 0; i < 2 * m; i++) {
			names[i] = "f_" + i;
		}
		f = cplex.intVarArray(2 * m, 0, k, names);

		// (4) flow from 0 to j is k
		IloLinearNumExpr constr1 = cplex.linearNumExpr();
		for (int e : instance.getIncidentEdges(0)) {
			if (instance.getEdge(e).getV1() == 0) {
				constr1.addTerm(1, f[e]);
			}
		}
		cplex.addEq(k, constr1);

		// (5) flow from j to 0 is 0
		IloLinearNumExpr constr2 = cplex.linearNumExpr();
		for (int e : instance.getIncidentEdges(0)) {
			if (instance.getEdge(e).getV1() == 0) {
				constr2.addTerm(1, f[e + m]);
			}
		}
		cplex.addEq(0, constr2);

		// (6) k - 1 selected edges
		IloLinearNumExpr constr2_5 = cplex.linearNumExpr();
		for (int e = 0; e < m; e++) {
			if (instance.getEdge(e).getV1() != 0
					&& instance.getEdge(e).getV2() != 0) {
				constr2_5.addTerm(1, x[e]);
				constr2_5.addTerm(1, x[e + m]);
			}
		}
		cplex.addEq(k - 1, constr2_5);

		// (7) v != 0, flow reduced by one on every visited node
		for (int v = 1; v < n; v++) {
			IloLinearNumExpr constr3 = cplex.linearNumExpr();
			IloLinearNumExpr constr4 = cplex.linearNumExpr();
			for (int e : instance.getIncidentEdges(v)) {
				if (instance.getEdge(e).getV1() == v) {
					constr3.addTerm(-1, f[e]);
					constr3.addTerm(1, f[e + m]);
					constr4.addTerm(1, f[e + m]);
				} else {
					constr3.addTerm(1, f[e]);
					constr3.addTerm(-1, f[e + m]);
					constr4.addTerm(1, f[e]);
				}
			}
			// if node not visited, flow not reduced
			cplex.addEq(constr3, cplex.min(1, constr4));
		}

		// (8) no flow on a not selected edge
		for (int e = 0; e < 2 * m; e++) {
			cplex.addLe(f[e], cplex.prod(k, x[e]));
		}

		// (9) if node is 0, then only one edge is selected
		// no backward edge to 0 is selected
		IloLinearNumExpr constr5 = cplex.linearNumExpr();
		IloLinearNumExpr constr6 = cplex.linearNumExpr();
		for (int e : instance.getIncidentEdges(0)) {
			if (instance.getEdge(e).getV1() == 0) {
				constr5.addTerm(1, x[e]);
				constr6.addTerm(1, x[e + m]);
			}
		}
		cplex.addEq(1, constr5);
		cplex.addEq(0, constr6);
	}

	private void modelMCF() throws IloException {
		// (1) edge variables
		x = cplex.boolVarArray(2 * m);

		// (2) objective function
		IloLinearIntExpr obj = cplex.linearIntExpr();
		for (int e = 0; e < m; e++) {
			int weight = this.instance.getEdge(e).getWeight();
			if (weight != 0) {
				obj.addTerm(x[e], weight);
				obj.addTerm(x[e + m], weight);
			}
		}
		cplex.addMinimize(obj);

		IloNumVar[][] g = new IloNumVar[n - 1][];
		for (int i = 0; i < n - 1; i++) {
			g[i] = cplex.numVarArray(2 * m, 0, 1);
		}
	}

	private void modelMTZ() throws IloException {
		// (1) edge variables
		x = cplex.boolVarArray(2 * m);

		// (2) objective function
		IloLinearIntExpr obj = cplex.linearIntExpr();
		for (int e = 0; e < m; e++) {
			int weight = this.instance.getEdge(e).getWeight();
			if (weight != 0) {
				obj.addTerm(x[e], weight);
				obj.addTerm(x[e + m], weight);
			}
		}
		cplex.addMinimize(obj);

		// u variable, order of a node, 0 <= u <= k
		IloIntVar[] u = cplex.intVarArray(n, 0, k);

		// node 0 has order 0
		cplex.addEq(u[0], 0);

		// 1 outgoing edge from node 0
		IloLinearNumExpr constr1 = cplex.linearNumExpr();
		for (int e : instance.getIncidentEdges(0)) {
			if (instance.getEdge(e).getV1() == 0) {
				constr1.addTerm(1, x[e]);
			}
		}
		cplex.addEq(constr1, 1);

		// k - 1 selected edges
		IloLinearNumExpr constr2 = cplex.linearNumExpr();
		for (int e = 0; e < m; e++) {
			if (instance.getEdge(e).getV1() != 0
					&& instance.getEdge(e).getV2() != 0) {
				constr2.addTerm(1, x[e]);
				constr2.addTerm(1, x[e + m]);
			}
		}
		cplex.addEq(k - 1, constr2);

		// sum of all u is k*(k+1)/2
		IloLinearNumExpr constr3 = cplex.linearNumExpr();
		for (int i = 0; i < n; i++) {
			constr3.addTerm(1, u[i]);
		}
		cplex.addEq(k * (k + 1) / 2, constr3);

		// node selected
		IloIntVar[] node = cplex.boolVarArray(n);

		// sum of selected nodes is k
		cplex.addEq(k, cplex.sum(node));
		
		// only give order to node if it is selected
		for(int i = 0; i < n; i++) {
			cplex.addLe(u[i], cplex.prod(k, node[i]));
		}

		// each node at most one incoming edge
		for (int j = 1; j < n; j++) {
			IloLinearNumExpr constr4 = cplex.linearNumExpr();
			for (int e : instance.getIncidentEdges(j)) {
				if (instance.getEdge(e).getV2() == j) {
					constr4.addTerm(1, x[e]);
				} else {
					constr4.addTerm(1, x[e + m]);
				}
			}
			cplex.addLe(constr4, 1);
		}

		// if an edge is selected, then both nodes have to have an order
		for (int e = 0; e < m; e++) {
			if (instance.getEdge(e).getV1() != 0) {
				cplex.addLe(x[e], u[instance.getEdge(e).getV1()]);
				cplex.addLe(x[e + m], u[instance.getEdge(e).getV1()]);
			}
			if (instance.getEdge(e).getV2() != 0) {
				cplex.addLe(x[e], u[instance.getEdge(e).getV2()]);
				cplex.addLe(x[e + m], u[instance.getEdge(e).getV2()]);
			}
		}
		
		// order of nodes, 
		for(int e = 0; e < m; e++) {
			IloLinearNumExpr constr4 = cplex.linearNumExpr();
			constr4.addTerm(1, u[instance.getEdge(e).getV1()]);
			constr4.addTerm(1, x[e]);

			IloLinearNumExpr constr5 = cplex.linearNumExpr(k);
			IloIntExpr constr6 = cplex.prod(k, x[e]);
			
			constr6.addTerm(1, u[instance.getEdge(e).getV2()]);
			
			cplex.addLe(constr4, constr6);
		}
	}
}
