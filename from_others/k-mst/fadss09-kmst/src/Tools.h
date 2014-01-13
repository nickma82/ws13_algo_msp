#ifndef __TOOLS__H__
#define __TOOLS__H__

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>

#include "Global.h"

extern int verbose;
#define LIST_SEP(i,m) ((i==m-1) ? "" : ", ")
#define LIST_SEP_NL(i,m) ((i==m-1) ? ";\n" : ", ")
#define COUT cout
#define COUT1 if (Global::inst()->verbose >= 1) cout
#define COUT2 if (Global::inst()->verbose >= 2) cout
#define COUT3 if (Global::inst()->verbose >= 3) cout
#define EOUT env.out()
#define EOUT1 if (Global::inst()->verbose >= 1) env.out()
#define EOUT2 if (Global::inst()->verbose >= 2) env.out()
#define EOUT3 if (Global::inst()->verbose >= 3) env.out()


using namespace std;

namespace Tools 
{


    // ---- color strings ----------------------------------
    /** terminal color constant */
    static const std::string RED     = "\033[0;31m\033[1m";
    /** terminal color constant */
    static const std::string BLUE    = "\033[0;34m\033[1m"; 
    /** terminal color constant */
    static const std::string BLACK   = "\033[0m";
    /** terminal color constant */
    static const std::string GREEN   = "\033[0;32m\033[1m"; 
    /** terminal color constant */
    static const std::string CYAN    = "\033[0;36m\033[1m";
    /** terminal color constant */
    static const std::string BROWN   = "\033[0;33m\033[1m";
    /** terminal color constant */
    static const std::string MAGENTA = "\033[0;35m\033[1m";
    /** terminal color constant */
    static const std::string GREY    = "\033[1;37m\033[1m";


    // ---- conversion functions ---------------------------

    /** converts integer to string */
    std::string itos(int i);

    std::vector<string> split(const std::string& s, const std::string& f);

    std::vector<int> stringToIntVector(std::string text);

    std::string indicesToString(std::string prefix, int i, int j);


    // ---- conversion functions ---------------------------

    /** measure running time */
    double CPUtime();

}; // Tools

#endif // __TOOLS__H__
