package kMinSpanningTreeCPLEX;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class Instance {
	/**
	 * array of edges
	 */
	private Edge[] edges;

	/**
	 * incident edges denoted by index in list edges
	 */
	private List<List<Integer>> incidentEdges;

	/**
	 * number of nodes
	 */
	private int numberOfNodes;

	/**
	 * number of edges
	 */
	private int numberOfEdges;

	public Instance(String file) {
		Path inputFile = Paths.get(file);
		if (!Files.exists(inputFile)) {
			System.err.println("could not open input file " + file);
			System.exit(-1);
		}

		System.out.println("Reading instance from file " + file);
		List<String> lines;
		try {
			lines = Files.readAllLines(inputFile, Charset.defaultCharset());

			// TODO add back edge for every edge with same weight
			
			numberOfNodes = Integer.parseInt(lines.remove(0));
			numberOfEdges = Integer.parseInt(lines.remove(0));

			edges = new Edge[numberOfEdges];

			// initialize incident edges array
			incidentEdges = new ArrayList<List<Integer>>();
			for (int i = 0; i < numberOfNodes; i++) {
				incidentEdges.add(new ArrayList<Integer>());
			}

			for (String line : lines) {
				if (!line.isEmpty()) {
					String[] values = line.split(" ");
					int id = Integer.parseInt(values[0]);
					int v1 = Integer.parseInt(values[1]);
					int v2 = Integer.parseInt(values[2]);
					int weight = Integer.parseInt(values[3]);

					// add new edge
					edges[id] = new Edge(v1, v2, weight);
					incidentEdges.get(edges[id].getV1()).add(id);
					incidentEdges.get(edges[id].getV2()).add(id);
				}
			}

			System.out.println("Incidency list:");
			for (int v = 0; v < numberOfNodes; v++) {
				System.out.print(v + ": ");

				for (int e : incidentEdges.get(v)) {
					System.out.print("(" + edges[e].getV1() + ","
							+ edges[e].getV2() + "), ");
				}
				System.out.println();
			}

		} catch (IOException e) {
			System.err.println(e.getMessage());
			System.exit(-1);
		}

	}

	public int getNumberOfNodes() {
		return numberOfNodes;
	}

	public int getNumberOfEdges() {
		return numberOfEdges;
	}

	public Edge getEdge(int i) {
		return edges[i];
	}

	public List<Integer> getIncidentEdges(int node) {
		return incidentEdges.get(node);
	}
}
