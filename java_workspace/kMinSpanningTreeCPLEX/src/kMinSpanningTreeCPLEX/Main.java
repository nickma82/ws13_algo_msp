package kMinSpanningTreeCPLEX;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.GnuParser;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;

public class Main {

	public static void main(String[] args) {
		// default values
		String file = "data/g06.dat";
		String model_type = "scf";
		int k = 40;

		// create Options object
		Options options = new Options();
		Option inputFile = new Option("f", "file", true, "filename");
		Option modelType = new Option("m", "model", true, "model type");
		Option nodesToConnect = new Option("k", true, "nodes to connect");

		options.addOption(inputFile);
		options.addOption(modelType);
		options.addOption(nodesToConnect);

		// create the CommandLineParser
		CommandLineParser parser = new GnuParser();
		CommandLine line;
		try {
			// parse the command line arguments
			line = parser.parse(options, args);

			for (Option o : line.getOptions()) {
				if (o.getOpt().equals(inputFile.getOpt())) {
					file = line.getOptionValue(inputFile.getOpt());
				} else if (o.getOpt().equals(modelType.getOpt())) {
					model_type = line.getOptionValue(modelType.getOpt());
				} else if (o.getOpt().equals(nodesToConnect.getOpt())) {
					try {
						k = Integer.parseInt(line.getOptionValue(nodesToConnect
								.getOpt()));
					} catch (NumberFormatException e) {
						System.err.println(k + " is not a valid integer. "
								+ e.getMessage());
						System.exit(-1);
					}
				} else {
					System.out
							.println("USAGE:\t\t<program> -f filename -m model [-k <nodes to connect>]");
					System.out
							.println("EXAMPLE:\t./kmst -f data/g01.dat -m scf -k 5");
					System.out.println();
					System.exit(1);
				}
			}

		} catch (ParseException exp) {
			System.err.println("Parsing failed.  Reason: " + exp.getMessage());
			System.exit(-1);
		}

		// read instance
		Instance instance = new Instance(file);
		// solve instance
		kMST_ILP ilp = new kMST_ILP(instance, model_type, k);
		ilp.solve();
	}
}
