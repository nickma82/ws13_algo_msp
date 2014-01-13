#ifndef __INSTANCE__H__
#define __INSTANCE__H__


/*******************************************************************************
 **
 **   filename    :  Instance.h
 **   author      :  andy
 **   date        :  2009-02-30
 ** 
 **   purpose     :  singleton class holding input data
 ** 
 **
 *******************************************************************************/

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <set>
#include <vector>


using namespace std;

typedef std::pair<u_int, u_int> E; // edge type


class Instance
{
 public:
  u_int nNodes;
  u_int nEdges;
  u_int nArcs;

  // nodes
  /*
  vector<int> maxCoords;
  vector<string> nodeNames;
  vector< vector<u_int> > nodeCoordinates;
  vector< vector<u_int> > nodeLabels;
  vector< vector<u_int> > labelNodes;
  */
  // edges
  vector<E> edges;
  vector<int> edgeWeights;
  vector< vector<u_int> > edgeLabels;
  vector< vector<u_int> > labelEdges;

  // arcs
  /*
  vector< pair<u_int,u_int> > arcs;
  vector< vector<u_int> > arcLabels;
  vector< vector<u_int> > labelArcs;
  */


  // data structures
  vector< set<u_int> > adjEdges; // containting for each vertex its adj edges
  
  
 private: // ----- data members ------------------------------------------------
  static Instance* pInstance;


  

 public: // ----- public functions ---------------------------------------------
  static Instance* inst();
  bool readData(string filename);
  set<u_int> getAdjNodes(u_int node);
  set<u_int> getOutEdges(u_int node);
  set<u_int> getInEdges(u_int node);

 protected: // ----- protected functions ---------------------------------------
  Instance();
  Instance(const Instance&);
  Instance& operator=(const Instance&);
  
  /** checks if labels are consecutive */
  void validateData();
  

}; // Instance

#endif //__INSTANCE__H__
