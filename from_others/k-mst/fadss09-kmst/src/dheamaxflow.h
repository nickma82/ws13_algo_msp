/* Maximal flow - Push-Relabel algorithm */
/* Queue with gap and global updates */
/* Boris Cherkassky - cher@theory.stanford.edu, on.cher@zib-berlin.de */
/* Andrew V. Goldberg - goldberg@cs.stanford.edu */
/* c++ wrapper made @ ads.tuwien.ac.at */
/* Stefan Slaby - e0127018@student.tuwien.ac.at */

#ifndef DHEAMAXFLOW_H
#define DHEAMAXFLOW_H

#include <utility>
#include <cstdlib>

struct node; // forward declaration because of circular dependency

struct arc
{
	double cap;      // capacity
	double r_cap;    // residual capacity
	int    idx;      // position in arc array + 1, *(-1) for auto-generated opposite arcs
	node   *tail;    // tail node
	arc    *sister;  // opposite arc
	arc    *next;    // next arc with the same head
};

struct node
{
	arc    *first;   // first outgoing arc
	arc    *current; // current incident arc
	double excess;   // excess of the node
	long   rank;     // distance from the sink
	node   *q_next;  // next node in queue
	node   *nl_prev; // used by prefl_to_flow
};

class DheaMaxFlow
{
public:

	// n = #nodes, m = #arcs
	// expects node indices to range from 0 to n-1
	// arcs are assumed to lead from node arc->first to node arc->second
	DheaMaxFlow(int n, int m, std::pair<int,int> *arcs);

	~DheaMaxFlow();

	// must be called at least once before using the algorithm
	// calling the algorithm first yields undefined behaviour
	// capacities must be in the same order as arcs have been passed to constructor
	void update(int s, int t, double *capacities);
	
	// use this to change source / target nodes without changing capacities
	void update(int s, int t);

	/* temporarily removed because it doesn't work
	   when reimplementing this, take into account that *arcs doesn't contain self-loops
	double max_flow(double **r_cap); */

	// returns capacity of the minimal cut (return value works only once per update()!)
	// will be performed only if value of minimal cut is < border
	// (*cut)[i] gets assigned the following values:
	//    0 if there are minimal cuts both with and without node i
	//    1 if node i is on source side of the first (and therefore each) minimal cut
	//    2 if node i is on target side of the last (and therefore each) minimal cut
	double min_cut(double border, int **cut);

private:
	// prevent compiler from auto-generating these
	DheaMaxFlow(const DheaMaxFlow& other);
	DheaMaxFlow& operator=(const DheaMaxFlow& other);

protected:
	int pr_init();
	void def_ranks();
	int push(node *i);
	long relabel(node *i);
	int prflow(double *fl);
	void prefl_to_flow();

	int    n;                    // #nodes
	int    m;                    // #arcs (input graph)
	int    num_arcs;             // #arcs with opposing arcs / without loops (<= 2m)

	node   *nodes;               // array of nodes
	arc    *arcs;                // array of arcs
	
	node   *nsource;             // origin (source node)
	node   *nsink;               // destination (target node)

	node   **queue;              // queue for storing nodes
	node   **q_read, **q_write;  // queue pointers
	node   *qp_first, *qp_last;  // start and end of push-queue

	long   lmax;                 // maximal layer

	// to ensure that there always is at least one path from source to sink (requirement
	// of the original algorithm), both update methods insert an artificial
	// arc (source, sink) with 0 capacity if it isn't on the lists. since this arc
	// has to be removed on later updates, this variable keeps track of it
	bool   artificial_arc;

	// statistics
	long n_up;
	long n_push;
	long n_rel;
};

#endif /* DHEAMAXFLOW_H */


