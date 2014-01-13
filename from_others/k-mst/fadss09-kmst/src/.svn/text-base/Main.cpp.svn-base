#ifndef __MAIN__CPP__
#define __MAIN__CPP__


#include<iostream>

#include "EdgeIO.h"
#include "Global.h"
#include "Instance.h"
#include "kMST_BC.h"
#include "kMST_BC_CutCallback.h"
#include "Tools.h"
#include "dheamaxflow.h"

using namespace std;


void usage() 
{
   cout << "USAGE:\t" << Global::inst()->appName 
	<< " -f filename -a algorithm [-k nodes to connect] [-v verbosity] \n";
   cout << "EXAMPLE:\t" << "./mlsp -f data/fad-01.dat -a flow -k 5\n";
   cout << endl;
   exit(1);
} // usage


int main(int argc, char *argv[]) 
{

   COUT2 << endl << endl << endl << Tools::RED;
   COUT2 << "   *****************************************************" << endl;
   COUT2 << "   *****************************************************" << endl;
   COUT2 << "   ****                                             ****" << endl;
   COUT1 << "   ****  k-Node Minimum Spanning Tree               ****" << endl;
   COUT2 << "   ****                                             ****" << endl;
   COUT2 << "   *****************************************************" << endl;
   COUT2 << "   *****************************************************" << endl;
   COUT2 << Tools::BLACK << endl << endl << endl;


   Global::inst()->appName = argv[0];
   int opt;
   while ((opt=getopt(argc, argv, "f:a:k:v:")) != EOF) {
      switch (opt) {
      case 'f': Global::inst()->inputfile = optarg; break;
      case 'a': Global::inst()->alg = optarg; break; // algorithm to use
      case 'k': Global::inst()->k = atoi(optarg); break; // nodes to connect
      case 'v': Global::inst()->verbose = atoi(optarg); break; // verbosity level
      default: usage(); break;
      }
   }
   Instance::inst()->readData(Global::inst()->inputfile);

   kMST_BC alg;
   alg.computeSolution();

   return 0;
} // main


#endif // __MAIN__CPP__
