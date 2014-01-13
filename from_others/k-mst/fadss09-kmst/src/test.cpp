// test app for the dheamaxflow algorithm as updated by stefan slaby 02/2008

#include "dheamaxflow.h"

#include <cstdlib>
#include <iostream>

using namespace std;

int main() {

	// create graph
	int n = 6;
	int m = 9;
	pair<int, int> arcs[] =
	{
		pair<int, int>(0, 1),
		pair<int, int>(0, 2),
		pair<int, int>(1, 2),
		pair<int, int>(2, 1),
		pair<int, int>(1, 3),
		pair<int, int>(2, 4),
		pair<int, int>(3, 4),
		pair<int, int>(3, 5),
		pair<int, int>(4, 5)
	};

	// init algorithm
	DheaMaxFlow algorithm(n, m, arcs);

	// use algorithm
	double capacities[] =
	{
		5.0,
		2.0,
		2.0,
		2.0,
		3.0,
		4.0,
		1.0,
		2.0,
		5.0
	};
	algorithm.update(0, 5, capacities);

	int* cut = (int*) calloc(n, sizeof(int));
	double f = algorithm.min_cut(100.0, &cut);

	// results
	cout << "fluss f = " << f << endl;
	for(int i = 0; i < n; i++)
	{
		cout << "cut[" << i << "] = " << cut[i] << endl;
	}
	free(cut);

	capacities[0] = 6.0;
	algorithm.update(0, 5, capacities);

	cut = (int*) calloc(n, sizeof(int));
	f = algorithm.min_cut(100.0, &cut);

	// results
	cout << "fluss f = " << f << endl;
	for(int i = 0; i < n; i++)
	{
		cout << "cut[" << i << "] = " << cut[i] << endl;
	}
	free(cut);

	cut = (int*) calloc(n, sizeof(int));
	f = algorithm.min_cut(100.0, &cut);

	// results
	cout << "fluss f = " << f << endl;
	for(int i = 0; i < n; i++)
	{
		cout << "cut[" << i << "] = " << cut[i] << endl;
	}
	free(cut);

	capacities[0] = 4.0;
	algorithm.update(0, 5, capacities);

	cut = (int*) calloc(n, sizeof(int));
	f = algorithm.min_cut(100.0, &cut);

	// results
	cout << "fluss f = " << f << endl;
	for(int i = 0; i < n; i++)
	{
		cout << "cut[" << i << "] = " << cut[i] << endl;
	}
	free(cut);

}


