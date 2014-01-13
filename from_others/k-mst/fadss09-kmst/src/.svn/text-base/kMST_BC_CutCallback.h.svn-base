#ifndef __K_MST_BC_CUT_CALLBACK__H__
#define __K_MST_BC_CUT_CALLBACK__H__

#include <ilcplex/ilocplex.h>


#include <vector>
#include <list>

using namespace std;


struct sp_node_s { // used for shortest path computation
  u_int node_id;
  int pred; // -1 when uninitialized
  int pred_edge_id; // -1 when uninitialized
  double path_length;

  bool operator () (sp_node_s s1, sp_node_s s2) 
  { 
    return s1.path_length < s2.path_length; 
  }

};

inline bool operator < (sp_node_s s1, sp_node_s s2)
{
  return s1.path_length < s2.path_length;
}

inline bool operator == (sp_node_s s1, sp_node_s s2)
{
  return s1.path_length == s2.path_length;
}

inline ostream& 
operator << (ostream& os, const sp_node_s& s) 
{
  os << s.node_id << ", " << s.pred 
     << ", " << s.pred_edge_id << ", " << s.path_length;
  return os;
}

inline ostream& 
operator << (ostream& os, sp_node_s* s) 
{
  os << s->node_id << ", " << s->pred 
     << ", " << s->pred_edge_id << ", " << s->path_length;
  return os;
}

struct sp_result_s { // result of shortest path computation
  list<u_int> path;
  double length;
};



class kMST_BC_CutCallback : 
public IloCplex::LazyConstraintCallbackI
{
 public:
  static double epInt;  /* small epsilon to cover numeric deviations
			   from pure integers, i.e. a real number r is
			   treated as integer if there is an integer i
			   such that:
			   
			   i +/- epInt == a */

 private:
  vector<double> edgeWeights; /* should be set according to current LP
				 solution (for shortest path
				 calculation) */

  // x, z .. to have access to the current values
  IloBoolVarArray x;
  IloBoolVarArray z;

 public:
  kMST_BC_CutCallback(IloEnv env, const IloBoolVarArray& x, 
		      const IloBoolVarArray& z);
	
  virtual ~kMST_BC_CutCallback();

  // need not to be used
  IloCplex::CallbackI* duplicateCallback() const;

  // the code for the separation routine goes in here
  void virtual main();

  static IloCplex::Callback create(IloEnv env, const IloBoolVarArray& x, 
				   const IloBoolVarArray& z);
	

 private:

  /** returns a list of edge id's of shortest path according to
      vector<int> edgeWeights */
  sp_result_s shortestPath(u_int source, u_int target);

};

#endif  // __K_MST_BC_CUT_CALLBACK__H__
