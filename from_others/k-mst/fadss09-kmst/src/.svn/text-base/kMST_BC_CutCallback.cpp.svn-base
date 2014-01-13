#include "kMST_BC_CutCallback.h"

#include "Instance.h"
#include "Tools.h"
#include "dheamaxflow.h"

#define INF std::numeric_limits<double>::infinity()

double kMST_BC_CutCallback::epInt;

kMST_BC_CutCallback::kMST_BC_CutCallback(IloEnv env, const IloBoolVarArray& x,
					 const IloBoolVarArray& z)
  : IloCplex::LazyConstraintCallbackI(env)
{
   this->edgeWeights = vector<double>(Instance::inst()->nEdges, 1.);
   this->x = x;
   this->z = z;
}


kMST_BC_CutCallback::~kMST_BC_CutCallback()
{
   this->edgeWeights.clear();
}


IloCplex::CallbackI*
kMST_BC_CutCallback::duplicateCallback() const
{
   return NULL;
}


void
kMST_BC_CutCallback::main()
{
	IloCplex cplex = IloCplex(getModel());
	epInt=cplex.getParam(IloCplex::EpInt);
   COUT1 << Tools::RED << " + + + + calling user cut procedure + + + + "
	<< Tools::BLACK << endl;

   for(int i=0; i < Instance::inst()->nEdges; i++){
	   IloNum a = getValue(x[i]);
	   if (a<=epInt){
		   edgeWeights[i]= numeric_limits<double>::max();
	   }
   }

   for(int i=0; i < Instance::inst()->nEdges; i++)
   {
	   IloNum xe = getValue(x[i]);

	   if (xe > 0 + epInt) {

		   edgeWeights[i]= numeric_limits<double>::max();

		   sp_result_s  res = shortestPath(Instance::inst()->edges[i].first, Instance::inst()->edges[i].second);

		   IloExpr sum(getEnv());
		   //u_int *p;
		   //p=res.path.get_allocator().allocate(res.path.size());
		 //  for (list<u_int>::iterator it = res.path.begin(); it != res.path.end(); it++)
		  //     cout << *it << " ";

		  int sum_int = 0;
		  //cout << "path size: " << res.length << "node1: " << Instance::inst()->edges[i].first << "node2: "<<Instance::inst()->edges[i].second<<endl;
		  if(res.path.size() > 0) {

			  sum += 1-x[i];
			  sum_int += 1-(int)xe;
			  for (list<u_int>::iterator it = res.path.begin(); it != res.path.end(); it++)
				{
					xe = getValue(x[*it]);
					sum += 1-x[*it];
					sum_int += 1-(int)xe;

				}


				if (sum_int < 1 + res.path.size()*epInt){
					IloConstraint con = (sum >= 1);
					add(con);
				}
		   }
		   edgeWeights[i]= 1.;
	   }
   }
   for(int i=0; i < Instance::inst()->nEdges; i++){
	   edgeWeights[i]= 1.;

  }


   // TODO your separation code goes in here
}


IloCplex::Callback
  kMST_BC_CutCallback::create(IloEnv env, const IloBoolVarArray& x,
			      const IloBoolVarArray& z)
{
	cout<<"create callback"<<flush;
  return (new (env) kMST_BC_CutCallback(env, x, z));
}


sp_result_s
kMST_BC_CutCallback::shortestPath(u_int source, u_int target)
{

	COUT1 << Tools::BROWN << "CALLING SHORTEST PATH. s="
	 << source << ", t=" << target << Tools::BLACK << endl;
   u_int nNodes = Instance::inst()->nNodes;
   if (source >= nNodes || target >= nNodes) {
      cerr << "invalid nodes: source: " << source << ", target: " << target
	<< ". #nodes:" << nNodes << endl;
      exit(1);
   }
   vector<sp_node_s> nodes;
   vector<bool> F(nNodes, false); // indicates which nodes are
				  // finished

   for (u_int i=0; i<nNodes; i++) {
      sp_node_s sp;
      sp.node_id = i;
      sp.pred = -1;
      sp.pred_edge_id = -1;
      sp.path_length = (i != source) ? numeric_limits<double>::max() : 0;
      nodes.push_back(sp);
   }



   for (u_int i=0; i<nNodes; i++) {
      sp_node_s u;
      u.path_length = numeric_limits<double>::max();
      for (vector<sp_node_s>::iterator it = nodes.begin();
	   it != nodes.end(); ++it)
      {
    	 //cout<< i <<" nodes-id: " << it->node_id << " " <<F.at(it->node_id)<<" path length: "<< it->path_length<<endl<<flush;
		 if (!F.at(it->node_id)) {
			if (it->path_length < u.path_length) {
			   u.node_id = it->node_id;
			   u.pred = -1;
			   u.pred_edge_id = -1;
			   u.path_length = it->path_length;
			}
		 }
      }

      if (u.path_length  >= numeric_limits<double>::max()-1.) break;

      F.at(u.node_id) = true; // this node is finished now
      set<u_int> adj = Instance::inst()->adjEdges[u.node_id];
      for (set<u_int>::iterator it=adj.begin(); it!=adj.end(); ++it) {
    	 // cout<<"node: "<<u.node_id<<" adj: "<<*it<<endl<<flush;
		 if (this->edgeWeights.at(*it) <= 1.) {
			E* e = &Instance::inst()->edges.at(*it);
			u_int adj_node;
			if (e->first == u.node_id) {
			   adj_node = e->second;
			} else if (e->second == u.node_id) {
			   adj_node = e->first;
			} else {
			   cerr << "error in shortest path algorithm" << endl;
			   exit(1);
			}

			if (!F.at(adj_node)) { // adjacent node, not yet finished

			   if (nodes.at(adj_node).path_length >
			   (nodes.at(u.node_id).path_length +
				this->edgeWeights.at(*it)) )
			   {
			  nodes.at(adj_node).path_length = nodes.at(u.node_id).path_length
			  + this->edgeWeights.at(*it);

			  nodes.at(adj_node).pred = u.node_id;
			  nodes.at(adj_node).pred_edge_id = *it;
			   }

			}
		 }
      } // for adj. nodes
   } // for all nodes
   sp_result_s spr;
   spr.length = 0.;

   int idx = target;
   while (idx != (int)source && idx != -1) {
     int edge_id = nodes.at(idx).pred_edge_id;
     if (edge_id < 0) break;

     spr.length += this->edgeWeights.at(edge_id);

     spr.path.push_back(edge_id);
     idx = nodes.at(idx).pred;
   }


   COUT1 << Tools::BROWN << "RETURNING PATH: ";

   for (list<u_int>::iterator it = spr.path.begin(); it != spr.path.end(); it++) {
	   COUT1 << *it << ", ";
   }
   COUT1 << " with value: " << spr.length;
   COUT1 << Tools::BLACK << endl;

   return spr;
}
