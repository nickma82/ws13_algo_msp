#include "Instance.h"

Instance::Instance( string file ) :
	n_nodes( 0 ), n_edges( 0 )
{
	ifstream ifs( file.c_str() );
	if( ifs.fail() ) {
		cerr << "could not open input file " << file << "\n";
		exit( -1 );
	}

	cout << "Reading instance from file " << file << "\n";

	ifs >> n_nodes >> n_edges;
	cout << "Number of nodes: " << n_nodes << "\n";
	cout << "Number of edges: " << n_edges << "\n";

	edges.resize( n_edges );
	incidentEdges.resize( n_nodes );

	unsigned int id;
	while( ifs >> id ) {
		ifs >> edges[id].v1 >> edges[id].v2 >> edges[id].weight;
		incidentEdges[edges[id].v1].push_back( id );
		incidentEdges[edges[id].v2].push_back( id );
	}
	ifs.close();

	cout << "Incidency list:" << "\n";
	for( unsigned int v = 0; v < n_nodes; v++ ) {
		cout << v << ": ";
		for( auto& e : incidentEdges[v] ) {
			cout << "(" << edges[e].v1 << "," << edges[e].v2 << "), ";
		}
		cout << "\n";
	}
}
