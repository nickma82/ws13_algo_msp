#ifndef __INSTANCE__H__
#define __INSTANCE__H__

/*******************************************************************************
 **
 **   filename    :  Instance.h
 **   author      :  andy, (updated by mario)
 **   date        :  2011-03-03
 ** 
 **   purpose     :  read input data and build minimal graph structure
 ** 
 **
 *******************************************************************************/

#include "Tools.h"
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <fstream>

using namespace std;

class Instance
{

public:

	struct Edge
	{
		u_int v1, v2;
		int weight;
	};

	// number of nodes and edges
	u_int n_nodes, n_edges;
	// array of edges
	vector<Edge> edges;
	// incident edges denoted by index in vector <edges>
	vector<list<u_int> > incidentEdges;

	// constructor
	Instance( string file );

};
// Instance

#endif //__INSTANCE__H__
