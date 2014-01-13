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
#include <algorithm>
#include <iomanip>
#include <sys/times.h>

using namespace std;

namespace Tools
{
	// generate string from edge indices
	string indicesToString( string prefix, int i, int j = -1, int v = -1 );
	// measure running time
	double CPUtime();
}
;
// Tools

#endif // __TOOLS__H__
