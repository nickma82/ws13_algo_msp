#ifndef __EDGE_IO__H__
#define __EDGE_IO__H__


/*******************************************************************************
 **
 **   filename    :  EdgeIO.h
 **   author      :  andy
 **   date(updt)  :  2009-04-08
 ** 
 **
 *******************************************************************************/

#include <vector>
#include <iostream>

#include "Tools.h"

using namespace std;

class EdgeIO
{
 public:
  int id;
  int source_vertex;
  int target_vertex;
  vector<int> properties; // weights, capacities, etc.
  vector<int> labels; // numeric labels e.g. for colors etc. 

 public:
  EdgeIO() { }
  EdgeIO(int id, int source, int target, int weight=1) {
    this->id = id;
    this->source_vertex = source;
    this->target_vertex = target;
    this->properties.push_back(weight);
  }

}; // EdgeIO


inline bool operator >> (string s, EdgeIO& e) 
{
  const vector<string> vstr = Tools::split(s, ";");
  if (vstr.size() != 4) return false;

  e.id = atoi(vstr.at(0).c_str());

  vector<int> vint = Tools::stringToIntVector(vstr.at(1));
  e.source_vertex = vint.at(0);
  e.target_vertex = vint.at(1);

  vint = Tools::stringToIntVector(vstr.at(2));
  e.properties = vector<int>(vint.size());
  copy(vint.begin(), vint.end(), e.properties.begin());

  vint = Tools::stringToIntVector(vstr.at(3));
  e.labels = vector<int>(vint.size());
  copy(vint.begin(), vint.end(), e.labels.begin());

  return true;  
}




inline ostream& operator << (ostream& os, const EdgeIO& e) 
{
  os << e.id << "; ";
  os << e.source_vertex << ", " << e.target_vertex << "; ";

  for (size_t i=0; i<e.properties.size(); i++) {
    os << e.properties[i] << LIST_SEP(i, e.properties.size());
  }
  os << "; ";

  for (size_t i=0; i<e.labels.size(); i++) {
    os << e.labels[i] << LIST_SEP(i, e.labels.size());
  }
  os << "; ";

  return os;

}

#endif // __EDGE_IO__H__
