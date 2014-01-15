#ifndef __K_MST_ILP__H__
#define __K_MST_ILP__H__

#include "Tools.h"
#include "Instance.h"
#include <ilcplex/ilocplex.h>
#include <fstream>
#include <stdint.h>
#include <sstream>      // std::ostringstream

ILOSTLBEGIN

class kMST_Solution {
public:
	kMST_Solution(std::string _testInstance, const int _k);

	/**
	 * get the solution as ostringstream
	 */
	std::string getResultStream();
	std::string getCompareableResultStream();

	// inputs
	std::string testInstance;
	int k;

	// results
	std::string cplexStatus;
	double cpuTime;
	double objectiveValue;
	unsigned int branchAndBoundNodes;

};

class kMST_ILP {
private:
	// input data
	Instance& instance;
	std::string model_type;
	int k;
	// number of edges and nodes including root node and root edges
	unsigned int m, n;
	uint16_t m_edges;

	IloEnv env;
	IloModel model;
	IloCplex cplex;

	//Local vars
	std::fstream outputFile;

	/**
	 * Builds single commodity flow model
	 */
	void modelSCF(bool makeFasterResults = true);

	void modelMCF();
	/**
	 * Miller-Tucker-Zemlin model
	 */
	void modelMTZ();

public:
	kMST_ILP( Instance& _instance, std::string _model_type, int _k );
	~kMST_ILP();
	kMST_Solution solve();

private:
	void setCPLEXParameters();
};
// kMST_ILP

#endif //__K_MST_ILP__H__
