package roadgraph;


import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

public class Node {

    private GeographicPoint location;
    private List<Edge> edges;

    public Node(GeographicPoint location) {
        edges = new LinkedList<>();
        this.location = location;
    }

    public GeographicPoint getLocation() {
        return location;
    }


    public void addEdge(Edge edge) {
        edges.add(edge);
    }

    public List<Edge> getEdges() {
        return edges;
    }

    public void setEdges(List<Edge> edges) {
        this.edges = edges;
    }

}
