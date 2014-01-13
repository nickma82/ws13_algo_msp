#ifndef __K_MST_BC__H__
#define __K_MST_BC__H__


#include <ilcplex/ilocplex.h>

typedef IloIntVarArray IntVarArray;
typedef IloNumVarArray NumVarArray;
typedef IloBoolVarArray BoolVarArray;

using namespace std;

ILOSTLBEGIN


class kMST_BC
{
  
 private:
  IloCplex cplex; 
  IloEnv env;
  IloModel model; 
  
  BoolVarArray x;                  /* edge selection variables */
  BoolVarArray z;                  /* node selection variables */
  IntVarArray f;                   /* flow variables */
  
  IloNumArray vals;                /* to store result values of x */

  double epInt;   
  
  
 public:
    kMST_BC();
    void computeSolution();

 private:
    void setCplexParameters();


}; // kMST_BC

#endif //__K_MST_BC__H__
