#ifndef __INSTANCE__H__
#define __INSTANCE__H__

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
		unsigned int v1, v2;
		int weight;
	};

	// number of nodes and edges
	unsigned int n_nodes, n_edges;
	// array of edges
	vector<Edge> edges;
	// incident edges denoted by index in vector <edges>
	vector<list<unsigned int> > incidentEdges;

	// constructor
	Instance( string file );

};
// Instance

#endif //__INSTANCE__H__
