package roadgraph;


import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

public class Node implements Comparable<Node> {

    private GeographicPoint location;
    private List<Edge> edges;
    private double distanceStart;
    private double distanceGoal;

    public Node(GeographicPoint location) {
        edges = new LinkedList<>();
        this.location = location;
        distanceStart = Double.POSITIVE_INFINITY;
        distanceGoal = Double.POSITIVE_INFINITY;
    }

    public double getDistanceStart() {
        return distanceStart;
    }

    public void setDistanceStart(double distanceStart) {
        this.distanceStart = distanceStart;
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

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public double getDistanceGoal() {
        return distanceGoal;
    }

    public void setDistanceGoal(double distanceGoal) {
        this.distanceGoal = distanceGoal;
    }

    @Override
    public int compareTo(Node o) {
        return Double.compare(distanceStart + distanceGoal, o.distanceStart + o.distanceGoal);
    }
}
