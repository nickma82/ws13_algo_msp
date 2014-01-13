#include <iostream>
#include "Tools.h"
#include "Instance.h"
#include "kMST_ILP.h"

// for tests
#include "greatest.h"
#include <cmath>

using namespace std;

TEST singleCommoditiveFlow_g02(void) {

	Instance instance( "data/g02.dat" );
	kMST_ILP *ilp;
	kMST_Solution solution("", 0);

	ilp = new kMST_ILP( instance, "scf", 4 ); //single commoditive flow
	solution = ilp->solve();
    ASSERT_EQ(373, std::floor(0.1 + solution.objectiveValue));
    delete ilp;

	ilp = new kMST_ILP( instance, "scf", 10 );
	solution = ilp->solve();
	ASSERT_EQ( 1390, std::floor(0.1 + solution.objectiveValue) );
	PASS();
}

TEST singleCommoditiveFlow_g04(void) {
	Instance instance( "data/g04.dat" );
	kMST_ILP *ilp;
	kMST_Solution solution("", 0);

	ilp = new kMST_ILP( instance, "scf", 14 ); //single commoditive flow
	solution = ilp->solve();
    ASSERT_EQ(909, std::floor(0.1 + solution.objectiveValue));
    delete ilp;

	ilp = new kMST_ILP( instance, "scf", 35 );
	solution = ilp->solve();
	ASSERT_EQ(3292, std::floor(0.1 + solution.objectiveValue));

	PASS();
}

TEST millerTuckerZemlin_g02(void) {

	Instance instance( "data/g02.dat" );
	kMST_ILP *ilp;
	kMST_Solution solution("", 0);

	ilp = new kMST_ILP( instance, "mtz", 4 ); //single commoditive flow
	solution = ilp->solve();
    ASSERT_EQ(373, std::floor(0.1 + solution.objectiveValue));
    delete ilp;

	ilp = new kMST_ILP( instance, "mtz", 10 );
	solution = ilp->solve();
	ASSERT_EQ( 1390, std::floor(0.1 + solution.objectiveValue) );
	PASS();
}

TEST millerTuckerZemlin_g04(void) {
	Instance instance( "data/g04.dat" );
	kMST_ILP *ilp;
	kMST_Solution solution("", 0);

	ilp = new kMST_ILP( instance, "mtz", 14 ); //single commoditive flow
	solution = ilp->solve();
    ASSERT_EQ(909, std::floor(0.1 + solution.objectiveValue));
    delete ilp;

	ilp = new kMST_ILP( instance, "mtz", 35 );
	solution = ilp->solve();
	ASSERT_EQ(3292, std::floor(0.1 + solution.objectiveValue));

	PASS();
}

SUITE(scf_suite) {
	RUN_TEST(singleCommoditiveFlow_g02);
	RUN_TEST(singleCommoditiveFlow_g04);
	RUN_TEST(millerTuckerZemlin_g02);
	RUN_TEST(millerTuckerZemlin_g04);
}

void usage()
{
	cout << "USAGE:\t<program> -f filename -m model [-k <nodes to connect> -a]\n";
	cout << "EXAMPLE:\t" << "./kmst -f data/g01.dat -m scf -k 5\n\n";
	exit( 1 );
} // usage

/* Add all the definitions that need to be in the test runner's main file. */
GREATEST_MAIN_DEFS();

int main( int argc, char *argv[] ) {
	// read parameters
	int opt;
	// default values
	string file( "data/g01.dat" );
	string model_type( "flow" );
	int k = 5;

	while( (opt = getopt( argc, argv, "f:m:k:" )) != EOF ) {
		switch( opt ) {
			case 'f': // instance file
				file = optarg;
				break;
			case 'm': // algorithm to use
				model_type = optarg;
				break;
			case 'k': // nodes to connect
				k = atoi( optarg );
				break;
			default:
				usage();
				break;
		}
	}

	if ( argc > 1 ) {
		// read instance
		Instance instance( file );
		// solve instance
		kMST_ILP ilp( instance, model_type, k );
		ilp.solve();

	} else {
		std::cout << "Testing enabled..." << std::endl;
		GREATEST_MAIN_BEGIN(); /* command-line arguments, initialization. */
		greatest_info.flags |= GREATEST_FLAG_VERBOSE;

		RUN_SUITE(scf_suite);
		GREATEST_MAIN_END(); /* display results */
	}

	return 0;
}
