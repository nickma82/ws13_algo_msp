#include "Global.h"


Global* Global::pGlobal = 0; // initialize pointer



Global* 
Global::inst() 
{
   if (pGlobal == 0) {  
      pGlobal = new Global();
   }
   return pGlobal;
}



Global::Global() 
{
   //cout << "Global::Global() : TEST" << endl; 
   appName = "MLSP";
   alg = "flow";
   inputfile = "data/fad-01.txt";
   verbose = 0;

   k = 5;
}
