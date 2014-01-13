/* Maximal flow - Push-Relabel algorithm */
/* Queue with gap and global updates */
/* Boris Cherkassky - cher@theory.stanford.edu, on.cher@zib-berlin.de */
/* Andrew V. Goldberg - goldberg@cs.stanford.edu */
/* c++ wrapper made @ ads.tuwien.ac.at */
/* Stefan Slaby - e0127018@student.tuwien.ac.at */

#include "dheamaxflow.h"

#include <iostream>
#include <queue>
#include <utility>
#include <limits>

#ifndef ROUND_EPS
#define ROUND_EPS 0.0001
#endif

#ifndef FLOW_ROUND_EPS
#define FLOW_ROUND_EPS 0.0001
#endif

#define BIGGEST_FLOW std::numeric_limits<double>::max()
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#define GLOB_UPDT_FREQ 1
#define WHITE 0
#define GREY 1
#define BLACK 2

#define FLOW(a) ( ( a )->cap - ( a )->r_cap )

// --- constructor --- //
DheaMaxFlow::DheaMaxFlow(int n, int m, std::pair<int,int> *arcs)
{
	this->n = n;
	this->m = m;
	
	// --- initialize long-term arrays that are required in constructor --- //
	this->nodes = (node*) calloc(n + 1, sizeof(node));
	// size n, one past end is a legal pointer
	this->arcs = (arc*) calloc(2*m + 2, sizeof(arc));
	// size <=2m + 2 for the artifical arc that might be inserted during update

	// --- initialize local helper arrays --- //
	int *counter = (int*) calloc(n+1, sizeof(int));
	int *end = (int*) calloc(n, sizeof(int));
	// used for linear sorting of arc structures to keep track of boundaries
	int *arc_head = (int*) calloc(2*m, sizeof(int));
	// head of each arc in this->arcs
	/* temporarily removed, causes a segfault in prefl_to_flow()
	arc **position = (arc**) calloc(n*n, sizeof(arc*));
	// quadratic memory is necessary to efficiently detect opposite arcs
	// for memory-efficiency with large-scale sparse graphs this could in a 
	// later implementation be replaced with a hashmap */ 

	if(!this->nodes || !this->arcs || !counter || !end || !arc_head/* || !position*/)
	{
		std::cerr << "Couldn't allocate memory!" << std::endl;
	}

	// --- initialize arcs array, detect opposite arcs --- //
	{
		arc *current_arc = this->arcs;
		int current_pos = 0;
		for(int index = 0; index < m; index++)
		{
			std::pair<int, int> *this_arc = arcs + index;
			int head = this_arc->first;
			int tail = this_arc->second;
			if(head == tail)
				continue; // omit self-loops

			/* temporarily removed, causes a segfault in prefl_to_flow()
			arc *other_arc = position[n * head + tail];
			if(other_arc) // arc + opposite arc have already been initialized
			{
				other_arc->idx = index + 1;
			}
			else*/
			{
				counter[head + 1]++; counter[tail + 1]++;
				// counter[i+1] = number of edges incident to node i

				//position[n * head + tail] = current_arc;
				current_arc->tail         = nodes + tail;
				current_arc->sister       = current_arc + 1;
				current_arc->idx          = index + 1;
				arc_head[current_pos]     = head;
				current_arc++; current_pos++;

				//position[head + n * tail] = current_arc;
				current_arc->tail         = nodes + head;
				current_arc->sister       = current_arc - 1;
				current_arc->idx          = -(index + 1);
				arc_head[current_pos]     = tail;
				current_arc++; current_pos++;
			}
		}
		this->num_arcs = current_pos;
	}
	// there is no artificial arc before the first call to update
	this->artificial_arc = false;

	// --- ordering arcs, linear-time algorithm --- //
	// the precise effect of the following code is to re-arrange the arcs 
	// such that all arcs with the same head are placed in a coherent block
	// and those blocks are ordered by head index (ascending)

	// now, counter[i+1] is the number of arcs outgoing from node i
	for(int i = 0; i < n; i++)
	{
		counter[i+1] += counter[i];
		end[i] = counter[i+1];
	}
	// now,	counter[i] is the position of the first outgoing arc from node i
	// after they would be ordered

	for(int i = 0; i < n-1; i++) // scanning all nodes except the last
	{
		for(int current_pos = counter[i]; current_pos < end[i]; current_pos++)
		{
			int head = arc_head[current_pos];
			while(head != i)
			{
				// the arc at current_pos is out of place.
				// switch it where it belongs, and repeat
				int other_pos = counter[head];
				arc *current_arc = this->arcs + current_pos;
				arc *other_arc = this->arcs + other_pos;

				// swap current_arc and other_arc
				node *tail_temp = other_arc->tail;
				other_arc->tail    = current_arc->tail;
				current_arc->tail  = tail_temp;

				int idx_temp     = other_arc->idx;
				other_arc->idx   = current_arc->idx;
				current_arc->idx = idx_temp;

				if(other_arc != current_arc->sister) {
					arc *arc_temp       = other_arc->sister;
					other_arc->sister   = current_arc->sister;
					current_arc->sister = arc_temp;
					current_arc->sister->sister = current_arc;
					other_arc->sister->sister   = other_arc;
				}

				arc_head[current_pos] = arc_head[other_pos];
				arc_head[other_pos] = head;
				counter[head]++;
				head = arc_head[current_pos];
			}
		}
		// all arcs outgoing from node i are in place
	}
	// arcs are ordered

	// --- free local memory --- //
	free(counter);
	free(end);
	free(arc_head);
	// free(position);

	// --- initialize remaining long-term arrays --- //
	this->queue = (node**) calloc(n, sizeof(node*));
	// queue

	if(!this->queue)
	{
		std::cerr << "Couldn't allocate memory!" << std::endl;
	}
} // end of constructor

// --- destructor --- //
DheaMaxFlow::~DheaMaxFlow()
{    
	if(nodes) free(nodes);
	if(arcs) free(arcs);
	if(queue) free(queue);
}

// --- change source/target nodes and arc capacities --- //
void DheaMaxFlow::update(int s, int t, double *capacities)
{
	// artificial_arc doesn't need to be checked! since the lists are rebuilt either way,
	// an artifical arc gets automatically removed if it was there

	this->nsource = nodes + s;
	this->nsink = nodes + t;

	// assign (potentially) new capacities
	for(arc* current_arc = arcs + num_arcs - 1; current_arc >= arcs; current_arc--)
	{
		if(current_arc->idx > 0)
			current_arc->r_cap = current_arc->cap = capacities[current_arc->idx - 1];
		else
			current_arc->r_cap = 0;
	}

	// build lists
	for(node* current_node = nodes + n - 1; current_node >= nodes; current_node--)
	{
		current_node->first = (arc*) NULL;
	}
	for(arc* current_arc = arcs + num_arcs - 1; current_arc >= arcs; current_arc--)
	{
		// omit edges with 0 capacity in both directions
		if(current_arc->cap > 0 || current_arc->sister->cap > 0)
		{
			node *head = current_arc->sister->tail;
			current_arc->next = head->first;
			head->first = current_arc;
		}
	}

	// check for direct connection (s, t)
	artificial_arc = true;
	for(arc* current_arc = nsource->first; current_arc != NULL; current_arc = current_arc->next)
	{
		if(current_arc->tail == nsink) // direct connection is already there
		{
			artificial_arc = false;
			break;
		}
	}
	// build artificial arc (and opposite) at end of arc array if necessary
	if(artificial_arc)
	{
		arc* current_arc = arcs + num_arcs;
		// build (nsource, nsink)
		current_arc->tail   = nsink;
		current_arc->sister = current_arc + 1;
		current_arc->cap    = 0.0;
		current_arc->r_cap  = 0.0;
		// insert at head of nsource's list
		current_arc->next   = nsource->first;
		nsource->first = current_arc;
		current_arc++;
		// build (nsink, nsource)
		current_arc->tail   = nsource;
		current_arc->sister = current_arc - 1;
		current_arc->cap    = 0.0;
		current_arc->r_cap  = 0.0;
		// insert at head of nsink's list
		current_arc->next   = nsink->first;
		nsink->first = current_arc;
	}

	/* debug output, should probably be en/disabled by some # macro
	std::cout << "arc array:" << std::endl;
	for(int i = 0; i < (artificial_arc ? num_arcs+2 : num_arcs); i++)
	{
		arc* a = this->arcs + i;
		std::cout
			<< i
			<< ": ("
			<< (a->sister->tail - nodes)
			<< ","
			<< (a->tail - nodes)
			<< ") idx("
			<< a->idx
			<< ") cap("
			<< a->r_cap
			<< "/"
			<< a->cap
			<< ") sister("
			<< (a->sister - this->arcs)
			<< ") next("
			<< ((a->next) ? (a->next - this->arcs) : -1)
			<< ")"
			<< std::endl;  
	}*/
	
} // end of update(int s, int t, double *capacities)

// --- change source/target nodes --- //
void DheaMaxFlow::update(int s, int t)
{
	// remove artificial arc from lists if necessary
	if(artificial_arc)
	{
		nsource->first = nsource->first->next;
		nsink->first = nsink->first->next;
	}

	this->nsource = nodes + s;
	this->nsink = nodes + t;

	// reset capacities
	for(arc* current_arc = arcs + num_arcs - 1; current_arc >= arcs; current_arc--)
	{
		current_arc->r_cap = current_arc->cap;
	}

	// check for direct connection (s, t)
	artificial_arc = true;
	for(arc* current_arc = nsource->first; current_arc != NULL; current_arc = current_arc->next)
	{
		if(current_arc->tail == nsink) // direct connection is already there
		{
			artificial_arc = false;
			break;
		}
	}
	// build artificial arc (and opposite) at end of arc array.
	if(artificial_arc)
	{
		arc* current_arc = arcs + num_arcs;
		// build (nsource, nsink)
		current_arc->tail   = nsink;
		current_arc->sister = current_arc + 1;
		current_arc->cap    = 0.0;
		current_arc->r_cap  = 0.0;
		// insert at head of nsource's list
		current_arc->next   = nsource->first;
		nsource->first = current_arc;
		current_arc++;
		// build (nsink, nsource)
		current_arc->tail   = nsource;
		current_arc->sister = current_arc - 1;
		current_arc->cap    = 0.0;
		current_arc->r_cap  = 0.0;
		// insert at head of nsink's list
		current_arc->next   = nsink->first;
		nsink->first = current_arc;
	}

} // end of update(int s, int t)

// --- find first and last minimal cuts --- //
double DheaMaxFlow::min_cut(double border, int **cut)
{
	double f = 0;
	int e = 0;

	if((e = prflow(&f)))
	{
		std::cout << "Error in max_flow:" << e << std::endl;
		return 0;
	}

	for(int j = 0; j < n; j++)
		(*cut)[j] = 0;

	if(f < border)
	{
		std::queue<int> Q;
		Q.push(nsource - nodes);
		(*cut)[nsource - nodes] = 1;

		while(!Q.empty())
		{
			node akt = nodes[Q.front()]; Q.pop();

			arc *a = akt.first;
			while(a)
			{
				if(a->r_cap > 0 && (*cut)[a->tail - nodes] == 0)
				{
					(*cut)[a->tail - nodes + 0] = 1;
					Q.push(a->tail - nodes);
				}
				a = a->next;
			}
		}

		Q.push(nsink - nodes);
		(*cut)[nsink - nodes] = 2;

		while(!Q.empty())
		{
			node akt = nodes[Q.front()]; Q.pop();

			arc * a = akt.first;
			while(a)
			{
				if(a->sister->r_cap > 0 && (*cut)[a->tail - nodes] == 0)
				{
					(*cut)[a->tail - nodes] = 2;
					Q.push(a->tail - nodes);
				}
				a = a->next;
			}
		}
	}

	return f;
} // end of min_cut(double border, int **cut)


// --- initialization --- //
int DheaMaxFlow::pr_init()
{
	for(node *i = nodes; i < nodes + n; i++)
		i->excess = 0;

	nsource->excess = BIGGEST_FLOW;

	lmax = n - 1;

	return 0;
} // end of pr_init()


// --- global rank update - breadth first search --- //
void DheaMaxFlow::def_ranks()
{
	n_up++; // statistics

	// initialization
	for(node *i = nodes; i < nodes + n; i++)
		i->rank = n;

	nsink->rank = 0;

	*queue = nsink;
	qp_first = qp_last = NULL;

	lmax = 0;

	// breadth first search
	for(q_read = queue, q_write = queue + 1; q_read != q_write; q_read++)
	{
		// scanning arcs incident to node i
		node *i = *q_read;
		long j_rank = i->rank + 1;

		for(arc *a = i->first; a != NULL; a = a->next)
		{
			node *j = a -> tail;

			if(j->rank == n) // j is not labelled
			{
				if(a->sister->r_cap > 0) // arc (j, i) is not saturated
				{
					j->rank = j_rank;
					j->current = j->first;

					if(j_rank > lmax)
						lmax = j_rank;

					if(j->excess > 0)
					{
						j->q_next = qp_first;
						if(qp_first == NULL)
							qp_last = j;
						qp_first = j;
					}

					*q_write = j; q_write ++; // put j  to scanning queue
				}
			}
		} // node i is scanned
	} // end of scanning queue
} // end of def_ranks()


// --- pushing flow from node i --- //
int DheaMaxFlow::push(node *i)
{
	arc *a; // current arc (i,j)

	long j_rank = (i -> rank) - 1; // rank of the next layer

	// scanning arcs outgoing from i
	for(a = i->current; a != NULL; a = a->next)
	{
		if(a->r_cap > 0) // a is not saturated
		{
			node *j = a->tail;

			if(j->rank == j_rank) // j belongs to the next layer
			{
				double fl = MIN( i->excess, a->r_cap );

				a        ->r_cap -= fl;
				a->sister->r_cap += fl;

				n_push++; // statistics

				if(j_rank > 0)
				{
					if(j->excess == 0) // before current push j had zero excess
					{
						// put j to the push-list

						if(qp_first != NULL)
							qp_last->q_next = j;
						else
							qp_first = j;

						qp_last = j;
						j->q_next = NULL;
					} // j->excess == 0
				} // j->rank > 0

				j->excess += fl;
				i->excess -= fl;

				if(i->excess == 0)
					break;

			} // j belongs to the next layer
		} // a is not saturated
	} // end of scanning arcs from i

	i->current = a;

	return ((a == NULL) ? 1 : 0);
} // end of push(node *i)


// --- relabelling node i --- //
long DheaMaxFlow::relabel(node *i)
{
	long j_rank; // minimal rank of a node available from j
	arc *a_j = NULL; // an arc which leads to the node with minimal rank

	n_rel++; // statistics

	i->rank = j_rank = n;

	// looking for a node with minimal rank available from i
	for(arc *a = i->first; a != NULL; a = a->next)
	{
		if(a->r_cap > 0)
		{
			node *j = a->tail;

			if(j->rank < j_rank)
			{
				j_rank = j->rank;
				a_j    = a;
			}
		}
	}

	j_rank++;
	if(j_rank < n)
	{
		// siting i into the manual

		i->rank    = j_rank;
		i->current = a_j;

		if(j_rank > lmax)
			lmax = j_rank;

	} // end of j_rank < n

	return j_rank;
} // end of relabel(node *i)


// --- organizer --- //
int DheaMaxFlow::prflow(double *fl)
{
	int cc = pr_init(); // condition code
	if(cc)
		return cc;

	def_ranks ();

	long n_r = 0; // the number of relabels

	// queue method
	while(qp_first != NULL) // main loop
	{
		if(n_r > GLOB_UPDT_FREQ * (float)n) // it is time for global update
		{
			def_ranks();
			n_r = 0;
			if(qp_first == NULL)
				break;
		}

		node *i = qp_first;
		qp_first = qp_first->q_next;
		if(qp_first == NULL)
			qp_last = NULL;

		long i_rank = i->rank;

		while(i_rank < n) 
		{
			// until i will be free from excess or beyond the gap
			if(push(i) == 0)
				break;

			// i must be relabeled
			i_rank = relabel(i);
			n_r++;
		} // end of scanning i
	} // end of the main loop

	*fl += nsink->excess;

	prefl_to_flow();

	return 0;
} // end of prflow(double *fl)


// --- removing excessive flow - second phase of PR-algorithm --- //
// do dfs in the reverse flow graph from nodes with excess
// cancel cycles if found
// return excess flow in topological order
// - rank is used for dfs labels
// - nl_prev is used for DFS tree
// - q_next is used for topological order list
void DheaMaxFlow::prefl_to_flow()
{
	/* debug output, should probably be en/disabled by some # macro
	std::cout << "node array at start of prefl_to_flow():" << std::endl;
	for(int i = 0; i < n; i++)
	{
		node* x = this->nodes + i;
		std::cout
			<< i
			<< ": first("
			<< (x->first - arcs)
			<< ") excess("
			<< x->excess
			<< ")"
			<< std::endl;  
	}
	std::cout << "arc array at start of prefl_to_flow():" << std::endl;
	for(int i = 0; i < (artificial_arc ? num_arcs+2 : num_arcs); i++)
	{
		arc* a = this->arcs + i;
		std::cout
			<< i
			<< ": ("
			<< (a->sister->tail - nodes)
			<< ","
			<< (a->tail - nodes)
			<< ") idx("
			<< a->idx
			<< ") cap("
			<< a->r_cap
			<< "/"
			<< a->cap
			<< ") sister("
			<< (a->sister - this->arcs)
			<< ") next("
			<< ((a->next) ? (a->next - this->arcs) : -1)
			<< ")"
			<< std::endl;
	}*/

	// initialize
	node *bos = NULL, *tos = NULL;
	for(node *i = nodes; i < nodes + n; i++)
	{
		i->rank = WHITE;
		i->nl_prev = NULL;
		i->current = i->first;
	}

	for(node *i = nodes; i < nodes + n; i++)
	{
		if((i->rank == WHITE) && (i->excess > 0) && (i != nsource) && (i != nsink))
		{
			node *r = i;
			r->rank = GREY;
			while(1)
			{
				for(/*no init*/; i->current != NULL; i->current = i->current->next)
				{
					arc *a = i->current;
					if((a->cap == 0) && (a->r_cap > 0) && (a->tail != nsource) && (a->tail != nsink))
					{
						node *j = a->tail;
						if(j->rank == WHITE)
						{
							// start scanning j
							j->rank = GREY;
							j->nl_prev = i;
							i = j;
							break;
						}
						else if(j->rank == GREY)
						{
							// find minimum flow on the cycle
							double delta = a->r_cap;
							while(1)
							{
								delta = MIN( delta, j->current->r_cap );
								if(j == i)
									break;
								else
									j = j->current->tail;
							}

							// remove delta flow units
							j = i;
							while(1)
							{
								a = j->current;
								a        ->r_cap -= delta;
								a->sister->r_cap += delta;
								j = a->tail;
								if(j == i)
									break;
							}

							// back DFS to the first zeroed arc
							node *restart = i;
							for(j = i->current->tail; j != i; j = a->tail)
							{
								a = j->current;
								if((j->rank == WHITE) || (a->r_cap == 0))
								{
									j->current->tail->rank = WHITE;
									if(j->rank != WHITE)
										restart = j;
								}
							}

							if (restart != i)
							{
								i = restart;
								i->current = i->current->next;
								break;
							}
						}
					}
				}

				if(i->current == NULL)
				{
					// scan of i complete
					i->rank = BLACK;
					if(i != nsource)
					{
						if(bos == NULL)
						{
							bos = i;
							tos = i;
						}
						else
						{
							i->q_next = tos;
							tos = i;
						}
					}

					if(i != r)
					{
						i = i->nl_prev;
						i->current = i->current->next;
					}
					else
						break;
				}
			}
		}
	}

	// return excesses
	// note that sink is not on the stack
	if(bos != NULL)
	{
		node *i = tos;
		while(1)
		{
			arc *a = i->first;

			// Inserted to avoid segfault!!
			if (a != NULL)
			{
			// Inserted to avoid segfault!!

				while(i->excess > FLOW_ROUND_EPS)
				{
					if(!a)
						std::cout << "a is NULL, Excess left is " << i->excess << std::endl;

					if((a->cap == 0) && (a->r_cap > 0 ))
					{
						double delta = MIN( i->excess, a->r_cap );
						a        ->r_cap -= delta;
						a->sister->r_cap += delta;
						i      ->excess -= delta;
						a->tail->excess += delta;
					}
					a = a->next;
				}
			// Inserted to avoid segfault!!      
			}
			// Inserted to avoid segfault!!

			if(i == bos)
				break;
			else
				i = i->q_next;
		}
	}
} // end of prefl_to_flow()


