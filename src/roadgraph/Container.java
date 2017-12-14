package roadgraph;

public class Container {
    private Node node;
    private double distance;

    public Container(Node node) {
        this.node = node;
        distance = 1700000000000d;
    }

    public Node getNode() {
        return node;
    }

    public void setNode(Node node) {
        this.node = node;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
