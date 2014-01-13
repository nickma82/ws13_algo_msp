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
		// edge variables
		IloIntVar[] x = cplex.boolVarArray(2 * m);
		
		// objective function
		IloLinearIntExpr obj = cplex.linearIntExpr();
		for(int e = 0; e < m; e++) {
			int weight = this.instance.getEdge(e).getWeight();
			obj.addTerm(x[e], weight);
			obj.addTerm(x[e + m], weight);
		}
		cplex.addMinimize(obj);
		
		
	}

	private void modelMCF() throws IloException {
		
	}

	private void modelMTZ() throws IloException {

	}
}
