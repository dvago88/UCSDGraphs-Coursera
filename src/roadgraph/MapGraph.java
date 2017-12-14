package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Daniel Vargas
 * <p>
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
public class MapGraph {
    private int numVertices;
    private int numEdges;
    private HashMap<GeographicPoint, Node> vertices;
    public static int contador = 0;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        vertices = new HashMap<>();
        numEdges = 0;
        numVertices = 0;
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return numVertices;
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return vertices.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return numEdges;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location != null && vertices.get(location) == null) {
            vertices.put(location, new Node(location));
            numVertices++;
            return true;
        }
        return false;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        if (vertices.get(from) == null || vertices.get(to) == null || roadName == null || roadType == null || length < 0) {
            throw new IllegalArgumentException();
        }
        Edge edge = new Edge(from, to, roadName, roadType, length);
        vertices.get(from).addEdge(edge);
        numEdges++;
    }


    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        LinkedList<GeographicPoint> cola = new LinkedList<>();
        LinkedList<GeographicPoint> path = new LinkedList<>();
        HashSet<GeographicPoint> revisador = new HashSet<>();
        HashMap<Node, Node> parents = new HashMap<>();

        cola.add(start);
        revisador.add(start);

        while (!cola.isEmpty()) {
            GeographicPoint actual = cola.pollFirst();
            nodeSearched.accept(actual);
            //Si lo encontro, reconstruya el camino
            if (pathConstructor(start, goal, path, parents, actual)) return path;
            List<Edge> edges = vertices.get(actual).getEdges();
            //Agrego cada uno de los vertices a la cola si aun no estaban:
            for (Edge edge : edges) {
                if (revisador.add(edge.getTo())) {
                    cola.add(edge.getTo());
                    // (Node hijo, Node papa)
                    parents.put(vertices.get(edge.getTo()), vertices.get(actual));
                }
            }
        }
        return null;
    }


    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        infiniter();
        contador = 0; // Este contador es para poder ver cuantos nodos visito
        PriorityQueue<Node> cola = new PriorityQueue<>();
        LinkedList<GeographicPoint> path = new LinkedList<>();
        HashSet<Node> revisador = new HashSet<>();
        HashMap<Node, Node> parents = new HashMap<>();

        vertices.get(start).setDistanceStart(0d);
        vertices.get(start).setDistanceGoal(0d);

        cola.add(vertices.get(start));
        while (!cola.isEmpty()) {
            Node actual = cola.poll();
            nodeSearched.accept(actual.getLocation());
            contador++;
//            Si no ha visitado a actual metalo y haga esto
            if (revisador.add(actual)) {

                nodeSearched.accept(actual.getLocation());
                GeographicPoint a = actual.getLocation();//Esto es para poder hacer pathConstructor para los 2 algoritmos
                //Si lo encontro, reconstruya el camino
                if (pathConstructor(start, goal, path, parents, a)) {
                    return path;
                }
                List<Edge> edges = actual.getEdges();

                for (Edge edge : edges) {
                    double dis = edge.getLength();
                    GeographicPoint other = edge.getTo();
                    Node hijo = vertices.get(other);
                    double dist = actual.getDistanceStart();
                    if (hijo.getDistanceStart() > dist + dis) {
                        hijo.setDistanceStart(dist + dis);
                        hijo.setDistanceGoal(0d);
                        cola.add(hijo);
                        parents.put(hijo, actual);
                    }
                }
            }
        }
        return path;
    }

    private void infiniter() {
        for (Node value : vertices.values()) {
            value.setDistanceGoal(Double.POSITIVE_INFINITY);
            value.setDistanceStart(Double.POSITIVE_INFINITY);
        }
    }

    private boolean pathConstructor(GeographicPoint start,
                                    GeographicPoint goal,
                                    LinkedList<GeographicPoint> path,
                                    HashMap<Node, Node> parents,
                                    GeographicPoint a) {
        if (a.equals(goal)) {
            path.add(goal);
            Node hijo = vertices.get(a);
            while (true) {
                Node papa = parents.get(hijo);
                path.addFirst(papa.getLocation());
                if (papa.getLocation().equals(start)) {
                    return true;
                }
                hijo = papa;
            }
        }
        return false;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        infiniter();
        contador = 0; // este contador es para poder ver cuantos nodos visito.
        PriorityQueue<Node> cola = new PriorityQueue<>();
        LinkedList<GeographicPoint> path = new LinkedList<>();
        HashSet<Node> revisador = new HashSet<>();
        HashMap<Node, Node> parents = new HashMap<>();

        vertices.get(start).setDistanceStart(0d);
        vertices.get(start).setDistanceGoal(0d);

        cola.add(vertices.get(start));
        while (!cola.isEmpty()) {
            Node actual = cola.poll();
            nodeSearched.accept(actual.getLocation());
            contador++;
//            Si no ha visitado a actual metalo y haga esto
            if (revisador.add(actual)) {

                nodeSearched.accept(actual.getLocation());
                GeographicPoint a = actual.getLocation();//Esto es para poder hacer pathConstructor para los 2 algoritmos
                //Si lo encontro, reconstruya el camino
                if (pathConstructor(start, goal, path, parents, a)) {
                    return path;
                }
                List<Edge> edges = actual.getEdges();

                for (Edge edge : edges) {
                    double disStart = edge.getLength();
                    GeographicPoint other = edge.getTo();
                    Node hijo = vertices.get(other);
                    double distStart = actual.getDistanceStart();
                    double distGoal = goal.distance(other);
                    if (hijo.getDistanceStart() + hijo.getDistanceGoal() > distStart + disStart + distGoal) {
                        hijo.setDistanceStart(distStart + disStart);
                        hijo.setDistanceGoal(distGoal);
                        cola.add(hijo);
                        parents.put(hijo, actual);
                    }
                }
            }
        }
        return path;
    }


    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("D:\\Coursera\\Diseño de Estructuras de Datos Orientado a Objetos Specialization\\Advance Data Structures\\UCSDGraphs\\data\\testdata\\simpletest.map", firstMap);
        System.out.println("DONE.");

        // You can use this method for testing.
       /* System.out.println("Vertices: " + firstMap.getNumVertices());
        System.out.println("Edges:" + firstMap.getNumEdges());
        GeographicPoint start = new GeographicPoint(1.0, 1.0);
        GeographicPoint goal = new GeographicPoint(8.0, -1.0);
        System.out.println(firstMap.getVertices());

        System.out.println(firstMap.bfs(start, goal));

        System.out.println(firstMap.dijkstra(start, goal));*/

        /* Here are some test cases you should try before you attempt
         * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */
        MapGraph simpleTestMap = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
        System.out.println(contador);
        List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);
        System.out.println(contador);


        MapGraph testMap = new MapGraph();
        GraphLoader.loadRoadMap("D:\\Coursera\\Diseño de Estructuras de Datos Orientado a Objetos Specialization\\Advance Data Structures\\UCSDGraphs\\data\\maps\\utc.map", testMap);

        // A very simple test using real data
        testStart = new GeographicPoint(32.869423, -117.220917);
        testEnd = new GeographicPoint(32.869255, -117.216927);
        System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
        testroute = testMap.dijkstra(testStart, testEnd);
        System.out.print("dijkstra: " + contador);
        testroute2 = testMap.aStarSearch(testStart, testEnd);
        System.out.println(" A*: " + contador);
        System.out.println("");


        // A slightly more complex test using real data
        testStart = new GeographicPoint(32.8674388, -117.2190213);
        testEnd = new GeographicPoint(32.8697828, -117.2244506);
        System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
        testroute = testMap.dijkstra(testStart, testEnd);
        System.out.print("dijkstra: " + contador);
        testroute2 = testMap.aStarSearch(testStart, testEnd);
        System.out.println(" A*: " + contador);
        System.out.println("");

		
		/* Use this code in Week 3 End of Week Quiz */
        /*MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

    }

}
