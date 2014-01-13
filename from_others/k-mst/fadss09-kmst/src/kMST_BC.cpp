
#include "kMST_BC.h"

#include "Tools.h"
#include "Instance.h"
#include "kMST_BC_CutCallback.h"


kMST_BC::kMST_BC()
{

}



void
kMST_BC::computeSolution()
{
	Instance *inst = Instance::inst();

	set<u_int>::iterator it;
	double t1, t2;
	t1 = Tools::CPUtime();
	if(Global::inst()->alg == "flow") {

		try
		{
			env = IloEnv();
			model=IloModel(env);

				// Initialization of x used in 8c
			//IloBoolVarArray x(env, inst->nEdges);
			IloIntVarArray x(env, inst->nEdges);

			for(u_int e=0; e<inst->nEdges; e++)
			{
				//stringstream myname;
				//myname << "x_" << i;

				//x[e] = IloBoolVar(env);
				x[e] = IloIntVar(env,(IloInt) 0,(IloInt) 1);
				//x[i].setName(myname.str());
			}

				// objective function
			IloExpr expr(env);
			for(int e=inst->nNodes-1; e < inst->nEdges; e++)
			{
				expr += inst->edgeWeights[e] * x[e];
			}

			model.add(IloMinimize(env, expr));
			expr.end();

				// Initialization of f used in 8b, 8c, 8d
			IloNumVarArray f(env, inst->nEdges*2);

			for(u_int e=0; e<inst->nEdges*2; e++)
			{
				stringstream myname;
				myname << "f_" << e;
				f[e] = IloNumVar(env,(IloInt) 0,(IloInt) (Global::inst()->k));
			}

			// - - - - - - expression 8b - - - - - -
			IloExpr ExprB(env);

			set<u_int> out = inst->getOutEdges(0);
			cout << "out size: " << out.size() << "\n" << flush;

			for( it=out.begin() ; it != out.end(); it++)
			{
				ExprB += f[*it];
				ExprB -= f[(*it)+inst->nEdges];
			}

			set<u_int> in = inst->getInEdges(0);
			//cout << "in size: " << in.size() << flush;

			for(it=in.begin() ; it != in.end(); it++)
			{
				ExprB -= f[*it];
				ExprB += f[(*it)+inst->nEdges];
			}

			model.add(ExprB == (IloInt) Global::inst()->k);

			ExprB.end();

				// - - - - - expression 8c - - - - -
			for(int i=1; i < inst->nNodes; i++)
			{
				IloExpr ExprC(env);
				IloExpr ExprC_2(env);

				set<u_int> in = inst->getInEdges(i);
				//cout << "in size: " << in.size() << flush;


				for(it=in.begin() ; it != in.end(); it++)
				{
					ExprC_2 += f[*it];
					ExprC += f[*it];
					ExprC -= f[(*it)+inst->nEdges];
				}

				set<u_int> out = inst->getOutEdges(i);
				//cout << "out size: " << out.size() << "\n" << flush;

				for( it=out.begin() ; it != out.end(); it++)
				{
					ExprC -= f[*it];
					ExprC += f[(*it)+inst->nEdges];
					ExprC_2 += f[(*it)+inst->nEdges];
				}

				model.add(ExprC == IloMin(1,ExprC_2));

				ExprC.end();
				ExprC_2.end();
			}



				// - - - - - expressions 8d and 8e - - - - -
			for(int e=0; e < inst->nEdges*2; e++)
			{
				IloExpr ExprD(env);

				ExprD += f[e];

				model.add(ExprD <= ((IloInt) Global::inst()->k) * x[e % inst->nEdges]);
				ExprD.end();
			}

				// - - - - - expression 8f - - - - -
			IloNumExpr ExprF(env);
			for(int e=inst->nNodes-1; e < inst->nEdges; e++)
			{
				ExprF += x[e];
			}

			model.add(ExprF == (Global::inst()->k - 1));
			ExprF.end();

			IloNumExpr ExprG(env);
			for(int e=0; e < inst->nNodes-1; e++)
			{
				ExprG += x[e];
			}

			model.add(ExprG == 1);
			ExprG.end();


			cplex = IloCplex(model);
			cplex.solve();


			IloAlgorithm::Status algStatus = cplex.getStatus();
			if(algStatus != IloAlgorithm::Optimal)
			{
				cout << "An error in solving has occurred. \n" << flush;
			} else
			{
				t2=Tools::CPUtime();
				env.out() << "time: " << t2-t1 <<endl;
				env.out() << "param size: " << cplex.getParam(IloCplex::EpInt) << endl;
					// model solved to optimality
				env.out() << "obj. value: " << cplex.getObjValue() << endl;

				IloNumArray vals1(env);

				cplex.getValues(vals1, x);
				env.out() << "Values x = " << vals1 << endl;
	//			cplex.getReducedCosts(vals1, x);
	//			env.out() << "Reduced Costs = " << vals1 << endl;

				IloNumArray vals2(env);

				cplex.getValues(vals2, f);
				env.out() << "Values f = " << vals2 << endl;

				cplex.exportModel("kmst.lp");

			}

		} catch(IloException& e)
		{
			cout << "There was an exception" << e << flush;
		}



	// - - - - - - - - - - CYCLE ELIMINATION CUT FORMULATION - - - - - - - - - -

	} else {

		try
		{
			env = IloEnv();
			model=IloModel(env);

				// Initialization of x used in (6a)
			IloBoolVarArray x(env, inst->nEdges);

			for(u_int e=0; e<inst->nEdges; e++)
			{
				//stringstream myname;
				//myname << "x_" << i;
				x[e] = IloBoolVar(env);
				//x[i].setName(myname.str());
			}

			IloBoolVarArray z(env, inst->nNodes);
			for(u_int i=0; i<inst->nNodes; i++)
			{
				z[i] = IloBoolVar(env);
			}


				// - - - - - objective function (6a) - - - - -
			IloExpr expr(env);
			for(int e=0; e < inst->nEdges; e++)
			{
				expr += inst->edgeWeights[e] * x[e];
			}

			model.add(IloMinimize(env, expr));
			expr.end();

				// - - - - - expression (6b) - - - - -
			IloNumExpr ExprB(env);
			for(int e=0; e < inst->nEdges; e++)
			{
				ExprB += x[e];
			}

			model.add(ExprB == (Global::inst()->k - 1));
			//model.add(ExprB == (inst->nNodes - 2));
			ExprB.end();

			// - - - - - expression (6c) - - - - -
			IloNumExpr ExprC(env);
			for(int e=0; e < inst->nNodes; e++)
			{
				ExprC += z[e];
			}

			model.add(ExprC == (Global::inst()->k));
			ExprC.end();

			// - - - - - expression (6d) - - - - -

			for(int e=0; e < inst->nEdges; e++)
			{
				IloNumExpr ExprD(env);
				ExprD += x[e];
				model.add(ExprD <= z[inst->edges[e].first]);
				ExprD.end();
			}

			for(int e=0; e < inst->nEdges; e++)
			{
				IloNumExpr ExprF(env);
				ExprF += x[e];
				model.add(ExprF <= z[inst->edges[e].second]);
				ExprF.end();
			}




			cplex = IloCplex(model);

			cplex.use(kMST_BC_CutCallback::create(env,x,z));
			cplex.solve();


			IloAlgorithm::Status algStatus = cplex.getStatus();
			if(algStatus != IloAlgorithm::Optimal)
			{
				cout << "An error in solving has occurred. \n" << flush;
			} else
			{	t2=Tools::CPUtime();
				env.out() << "time: " << t2-t1 <<endl;
				env.out() << "param size: " << cplex.getParam(IloCplex::EpInt) << endl;
					// model solved to optimality
				env.out() << "obj. value: " << cplex.getObjValue() << endl;


				IloNumArray vals1(env);

				cplex.getValues(vals1, x);
				env.out() << "Values x = " << vals1 << endl;
	//			cplex.getReducedCosts(vals1, x);
	//			env.out() << "Reduced Costs = " << vals1 << endl;

				cplex.exportModel("kmst.lp");

			}
		} catch(IloException& e)
		{
			cout << "There was an exception" << e << flush;
		}
	}
} // computeSolution()



// ----- private methods -----------------------------------------------


void
kMST_BC::setCplexParameters()
{
   COUT1 << endl << Tools::MAGENTA;
   COUT1 << "disabling cplex cuts" << endl;
   COUT1 << Tools::BLACK << endl;


   // disabling these cuts sometimes slows down the overall process,
   // but at least with prior versions some errors occured sometimes
   // --> by disabling them we are on the save side
   cplex.setParam(IloCplex::Covers, -1);
   cplex.setParam(IloCplex::Cliques, -1);
   cplex.setParam(IloCplex::DisjCuts, -1);
   cplex.setParam(IloCplex::FlowCovers, -1);
   cplex.setParam(IloCplex::FlowPaths, -1);

   cplex.setParam(IloCplex::FracCuts, -1);
   cplex.setParam(IloCplex::GUBCovers, -1);
   cplex.setParam(IloCplex::ImplBd, -1);
   cplex.setParam(IloCplex::MIRCuts, -1);

   cplex.setParam(IloCplex::AggCutLim, 10000);
   cplex.setParam(IloCplex::CutPass, -1); // 0 = auto

} // setCplexParameters

/*double CPUtime()
{
   tms t;
   times(&t);
   double ct = sysconf(_SC_CLK_TCK);
   return t.tms_utime/ct;
}*/
