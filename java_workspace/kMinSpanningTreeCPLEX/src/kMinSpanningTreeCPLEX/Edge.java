package kMinSpanningTreeCPLEX;

public class Edge {
	private int v1, v2;
	private int weight;
	
	public Edge(int v1, int v2, int weight) {
		this.v1 = v1;
		this.v2 = v2;
		this.weight = weight;
	}
	
	public int getV1() {
		return v1;
	}
	
	public int getV2() {
		return v2;
	}
	
	public int getWeight() {
		return weight;
	}

	@Override
	public String toString() {
		return "{" + this.v1 + "," + this.v2 + "}";
	}

	public Edge getBackEdge() {
		return new Edge(v2, v1, weight);
	}
}
