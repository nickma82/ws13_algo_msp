#include "Instance.h"
#include "Tools.h" 

#include <string>
#include <fstream>

#include "EdgeIO.h"

#define REMARKS 10
#define METADATA 20 
#define NODES 30
#define EDGES 40
#define ARCS 50

// instance output operator
#define IOUT if (false) cout

Instance* Instance::pInstance = 0; // initialize pointer




Instance* 
Instance::inst() 
{
   if (pInstance == 0) {  
      pInstance = new Instance();
   }
   return pInstance;
}


bool
Instance::readData(string filename)
{
   int status = 0;
   string line;
   ifstream ifs(filename.c_str());
   if (ifs.fail()) {
      cerr << "\ncould not open input file " << filename << endl;
      return false;
   }

   while (getline(ifs, line)) {
      IOUT << "line: " << line << endl;

      if (line.compare("# remarks")==0) status = REMARKS;
      else if (line.compare("# metadata")==0) status = METADATA;
      else if (line.compare("# edges")==0) status = EDGES;

      switch (status) {
      case METADATA: 
	 {
	    IOUT << "___METADATA___" << endl; 
	    getline(ifs, line);
	    nNodes = atoi(line.c_str());
	    getline(ifs, line);	   
	    nEdges = atoi(line.c_str());
	    getline(ifs, line);
	    
	    adjEdges = vector< set<u_int> >(nNodes);
	    IOUT << Tools::RED << "creating adjEdges vector with " 
		 << nNodes << " elements. " << Tools::BLACK << endl;
	    for (u_int i=0; i<nNodes; i++) {
	       adjEdges[i] = set<u_int>();
	    }

	    IOUT << "nNodes: " << nNodes << endl;
	    IOUT << "nEdges: " << nEdges << endl;
	    IOUT << "___METADATA__END___" << endl;
	    break;
	 }
      case EDGES:
	 {	   
	    IOUT << "___EDGES__" << endl; 
	    EdgeIO e;
	    if (line >> e) {
	       IOUT << "read edge: " << e << endl;
	       pair<u_int, u_int> edge(e.source_vertex, e.target_vertex);
	       edges.push_back(edge); 
	       edgeWeights.push_back(e.properties.at(0)); // weight
	       IOUT << "nNodes: " << nNodes << endl;
	       IOUT << "adjEdges.size() " << adjEdges.size() << endl;
	       adjEdges[e.source_vertex].insert(edges.size()-1);
	       adjEdges[e.target_vertex].insert(edges.size()-1);	       
	    } else {
	       IOUT << "couldn't read edge" << endl;
	    }
	    IOUT << "___EDGES__END___" << endl;
	    break;
	 }
      default: cerr << "invalid status " << status 
		    << "while reading input file" << endl;
	 break;
      }
      
   }
   ifs.close();
   
   
   
   IOUT << "PRINT ADJ LIST" << endl;
   for (u_int i=0; i<nNodes; i++) {
      IOUT << i << ": ";
      for (set<u_int>::iterator it=adjEdges[i].begin();
	   it!=adjEdges[i].end(); ++it) {
	 pair<u_int, u_int> p = edges[*it];
	 IOUT << "(" << p.first 
	      << "," << p.second << ") ,";
      }      
      IOUT << endl;
      set<u_int> a = getAdjNodes(i);
      for (set<u_int>::iterator it=a.begin(); it!=a.end(); ++it) {
	 IOUT << *it << ", ";
      }
      IOUT << endl;
   }
   return true;
} // readData(...)


set<u_int>
Instance::getAdjNodes(u_int node)
{
   if (node >= adjEdges.size() ) {
      cerr << "Instance::getAdjNodes called for invalid node " << node << endl;
      exit(1);
   }
   set<u_int> adj;

   for (set<u_int>::iterator it=adjEdges[node].begin();
	it != adjEdges[node].end(); ++it) 
   {
	 pair<u_int, u_int> p = edges[*it];
      adj.insert(p.second);
      adj.insert(p.first);
   }
   adj.erase(adj.find(node));
   return adj;
} // getAdjNodes(...)


set<u_int>
Instance::getOutEdges(u_int node) 
{
   if (node >= adjEdges.size() ) {
      cerr << "Instance::getOutEdges called for invalid node " << node << endl;
      exit(1);
   }
   set<u_int> outEdges;
   for (set<u_int>::iterator it=adjEdges[node].begin();
	it != adjEdges[node].end(); ++it) 
   {
      pair<u_int, u_int> p = edges[*it];
      if (p.first == node) outEdges.insert(*it);
   }
   return outEdges;
} // getOutEdges


set<u_int>
Instance::getInEdges(u_int node) 
{
   if (node >= adjEdges.size() ) {
      cout << "Instance::getInEdges called for invalid node " << node << endl;
      exit(1);
   }
   set<u_int> inEdges;
   for (set<u_int>::iterator it=adjEdges[node].begin();
	it != adjEdges[node].end(); ++it) 
   {
      pair<u_int, u_int> p = edges[*it];
      if (p.second == node) inEdges.insert(*it);
   }
   return inEdges;
} // getInEdges


// ----- private member functions ----------------------------


Instance::Instance() { }
