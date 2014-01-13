
#ifndef __GLOBAL__H__
#define __GLOBAL__H__


/*******************************************************************************
 **
 **   filename    :  Global.h
 **   author      :  andy
 **   date        :  2009-02-30
 ** 
 **   purpose     :  singleton class holding global values and parameters
 ** 
 **
 *******************************************************************************/

#include <string>
#include <iostream>

using namespace std;


class Global
{
  
 public: // ----- public data members ------------------------------------------
  string appName;
  string alg;
  string inputfile;
  u_int verbose;
  u_int k;

 private: // ----- data members ------------------------------------------------
  static Global* pGlobal;
  

  
 public: // ----- public functions ---------------------------------------------
  static Global* inst();

 protected: // ----- protected functions ---------------------------------------
  Global();
  Global(const Global&);
  Global& operator=(const Global&);
  
 
  

}; // Global

#endif //__GLOBAL__H__
