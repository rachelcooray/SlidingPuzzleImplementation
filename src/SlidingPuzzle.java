// Importing required libraries
import java.util.Scanner;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.HashSet;
import java.util.Comparator;

/**
 * The main class to solve the sliding puzzle using A* algorithm.
 */
public class SlidingPuzzle {
    /**
     * The main method to run the sliding puzzle solver.
     * It reads a grid map from the text file, processes it to a graph, finds a path from start to finish, and prints the path along with the time taken to complete the solution.
     */
    public static void main(String[] args) {
        // Scanner for user input
        Scanner scanner = new Scanner(System.in);

        // Get file path from user
        System.out.println("SLIDING PUZZLES\nEnter the file path: ");
        String filePath = scanner.nextLine();

        System.out.println("Running text file: " + filePath + "\n");

        // Start timing
        long startTime = System.nanoTime();

        try {
            // Parse the map and read content
            GridGraph gridGraph = MapParser.parseMap(filePath);
            String fileContent = readFileContent(filePath);

            // Print file content
            System.out.println("Content of " + filePath + ":");
            System.out.println(fileContent);

            int[][] grid = gridGraph.getGrid();
            int startRow = -1, startCol = -1, endRow = -1, endCol = -1;

            // for loop to get start and end points
            for (int i = 0; i < grid.length; i++) {
                for (int j = 0; j < grid[0].length; j++) {
                    if (grid[i][j] == 2) {
                        startRow = i;
                        startCol = j;
                    } else if (grid[i][j] == 3) {
                        endRow = i;
                        endCol = j;
                    }
                }
            }

            // Print start and end points
            System.out.println("\nStart point (S) = (" + (startRow + 1) + "," + (startCol + 1) + ")");
            System.out.println("End point (F) = (" + (endRow + 1) + "," + (endCol + 1) + ")\n");

            // Find the path
            List<String> path = AStar.findPath(gridGraph);

            System.out.println("\nThe steps to reach F from S:\n");

            // for loop to iterate over the steps in the path
            for (int i = 0; i < path.size(); i++) {
                System.out.println((i+1) + ". "+ path.get(i));
            }

        } catch (FileNotFoundException e) { // Catch block to handle FileNotFoundException
            System.err.println("File not found: " + e.getMessage());
        } catch (IndexOutOfBoundsException e) { // Catch block to handle dimension mismatches
            System.err.println("Error in mismatch of grid dimensions: " + e.getMessage());
        } catch (Exception e) { // Catch block to handle other exceptions
            System.err.println("An unexpected error occurred: " + e.getMessage());
        } finally {
            scanner.close();
        }

        long endTime = System.nanoTime(); // End timing
        double duration = (endTime - startTime) / 1e9; // Converting nanoseconds to seconds

        // Print the completion time
        System.out.println("\nCompletion time in seconds: " + duration);
        System.out.println("\nCompletion time in milli-seconds: " + duration * 1000);

        scanner.close();
    }

    /**
     * This method reads the content of a text file.
     * Take the path to the text file as a parameter and returns the content of the text file as a string.
     */
    private static String readFileContent(String filePath) throws FileNotFoundException {
        // Store the content of the file
        StringBuilder content = new StringBuilder();
        Scanner scanner = new Scanner(new File(filePath));

        while (scanner.hasNextLine()) {
            // Append the current line to content
            content.append(scanner.nextLine()).append("\n");
        }
        scanner.close();

        // Return the content of the file as a trimmed string
        return content.toString().trim();
    }
}


/**
 * This cl;ass represents a grid graph with a specific structure for the sliding puzzle.
 */
class GridGraph {
    private final int[][] grid; // 2D array representing the grid
    private final int width; // Width of the grid
    private final int height; // Height of the grid

    /*
     * Constructor for GridGraph with a specified grid.
     */
    public GridGraph(int[][] grid) {
        this.grid = grid;
        this.width = grid.length;
        this.height = grid[0].length; // Takes first int[] in the grid
    }

    // Getters for width, height, and grid
    public int getWidth() {

        return width;
    }

    public int getHeight() {

        return height;
    }

    public int[][] getGrid() {

        return grid;
    }

}

/**
 * This class is used to parse maps from files into GridGraph objects.
 */
class MapParser {

    /**
     * This method parses the specified file into a GridGraph object.
     * Takes the name of the file containing the map as a parameter and returns a GridGraph representing the map.
     */
    public static GridGraph parseMap(String filename) throws FileNotFoundException {
        Scanner scanner = new Scanner(new File(filename));

        int width = scanner.nextLine().length(); // Get the length of the first line as width
        int height = 1; // Set the first row as 1

        while (scanner.hasNextLine()) { // Iterate through each line of the file
            height++;
            scanner.nextLine();
        }
        scanner.close();

        int[][] grid = new int[width][height]; // Initialize grid with width and height
        scanner = new Scanner(new File(filename));
        int row = 0;
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine();
            if (line.isEmpty()) { // Skip empty lines
                continue;
            }
            height++; // Increment height counter

            for (int col = 0; col < width; col++) {
                char character = line.charAt(col); // Character at the current position
                if (character == '.') {
                    grid[col][row] = 1; // Represent ice with 1
                } else if (character == '0') {
                    grid[col][row] = 0; // Represent rocks with 0
                } else if (character == 'S') {
                    grid[col][row] = 2; // Represent start with 2
                } else if (character == 'F') {
                    grid[col][row] = 3; // Represent finish with 3
                } else {
                    System.out.println("Invalid character in file at: (" + width + " , " + height + ") ");
                }
            }
            row++; // Move to next row
        }
        scanner.close();

        return new GridGraph(grid); // Return the constructed GridGraph
    }
}

/**
 * This class is used for finding paths in grid graphs using A* search algorithm.
 */
class AStar {

    /**
     * Represents a node in the search space of the A* algorithm.
     */
    static class Node {
        // Variables of each node
        int x, y, fValue, gValue, hValue; // Coordinates, costs, and heuristic value
        Node parent; // Parent node

        /**
         * Constructor for a node with specified x,y coordinates passed in the parameter.
         */
        Node(int x, int y) {
            this.x = x; // Initialize the x-coordinate of the node
            this.y = y; // Initialize the y-coordinate of the node
        }
    }

    /**
     * This method finds the shortest path from the start to the finish in the given grid graph using the A* algorithm.
     * Takes the GridGraph to find the path in as a parameter and returns a list of steps representing the path from start to finish.
     */
    public static List<String> findPath(GridGraph grid) {
        int[][] graph = grid.getGrid();
        int startX = -1, startY = -1, endX = -1, endY = -1;

        // Set up start and finish points
        for (int i = 0; i < grid.getWidth(); i++) {
            for (int j = 0; j < grid.getHeight(); j++) {
                if (graph[i][j] == 2) { // If it's the start point(S)
                    // Set x,y coordinates of the start point
                    startX = i;
                    startY = j;
                } else if (graph[i][j] == 3) { // If it's the end point(F)
                    // Set x,y coordinates of the end point
                    endX = i;
                    endY = j;
                }
            }
        }

        List<String> steps = new ArrayList<>(); // List to store the steps of the path

        if (startX == -1 || startY == -1 || endX == -1 || endY == -1) {
            steps.add("Start or end point not found!"); // If start or end point not found
            return steps;
        }

        // Initialize open list, closed list, and nodes array
        PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingInt(node -> node.fValue)); // Priority foe node with least f-value
        Set<Node> closedList = new HashSet<>(); // Visited nodes
        Node[][] nodes = new Node[grid.getWidth()][grid.getHeight()];

        for (int i = 0; i < grid.getWidth(); i++) {
            for (int j = 0; j < grid.getHeight(); j++) {
                nodes[i][j] = new Node(i, j); // Create node objects for each cell in the grid
            }
        }

        Node startNode = nodes[startX][startY];
        Node endNode = nodes[endX][endY];

        openList.add(startNode);

        while (!openList.isEmpty()) {
            Node currentNode = openList.poll(); // Get the node with the lowest fValue (front of queue) from the open list
            closedList.add(currentNode);

            if (currentNode == endNode) {
                Node node = currentNode;

                // Reconstruct the path by tracing back from the end node to the start node
                while (node.parent != null) {
                    String direction = getDirection(node, node.parent); // Get the direction from the current node to its parent
                    steps.add(0, String.format("%s to (%d,%d)", direction, node.x + 1, node.y + 1)); // Add the direction to the steps list
                    node = node.parent; // Move to the parent node

                }
                steps.add(0, String.format("Start at (%d,%d)", startX + 1, startY + 1)); // Add start node to the steps list
                steps.add("Done!");
                return steps;
            }

            List<Node> neighbors = getNeighbors(currentNode, nodes, graph, grid.getWidth(), grid.getHeight()); // Get the neighboring nodes of the current node

            for (Node neighbor : neighbors) {
                if (closedList.contains(neighbor)) {
                    continue;
                }

                int newG = currentNode.gValue + 1;

                if (openList.contains(neighbor) && newG >= neighbor.gValue) {
                    continue;
                }

                neighbor.parent = currentNode;
                neighbor.gValue = newG;
                neighbor.hValue = heuristic(neighbor, endNode); // Heuristic value for the neighbor node
                neighbor.fValue = neighbor.gValue + neighbor.hValue; // fValue for the neighbor node

                if (!openList.contains(neighbor)) {
                    openList.add(neighbor); // Add the neighbor to the open list if it's not there alraedy
                }
            }
        }

        steps.add("No path was found.");
        return steps;
    }

    /**
     * This method compares the x and y coordinates of the current node with its parent to get the direction of movement.
     */
    private static String getDirection(Node current, Node parent) {
        if (current.x < parent.x) {
            return "Move left";
        } else if (current.x > parent.x) {
            return "Move right";
        } else if (current.y < parent.y) {
            return "Move up";
        } else if (current.y > parent.y){
            return "Move down";
        } else {
            return "No movement.";
        }
    }

    /**
     * Returns the heuristic value calculated using the Manhattan distance between two nodes.
     */
    private static int heuristic(Node a, Node b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y); // Manhattan distance
    }

    /**
     * Gets the neighboring nodes of a node that can be traversed.
     * Takes the current node, array of nodes, grid, width and height as parameters and returns a list of traversable neighboring nodes.
     */
    private static List<Node> getNeighbors(Node node, Node[][] nodes, int[][] graph, int width, int height) {
        List<Node> neighbors = new ArrayList<>();
        int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Movements in the order left, right, up, down

        for (int[] direction : directions) {
            int nextX = node.x + direction[0];
            int nextY = node.y + direction[1];

            if (nextX >= 0 && nextX < width && nextY >= 0 && nextY < height && graph[nextX][nextY] != 0) {
                neighbors.add(nodes[nextX][nextY]); // Add the neighbor to the list if it's in the grid and not blocked
            }
        }

        return neighbors;
    }

// SLIDING

//    private static List<Node> getNeighbors(Node node, Node[][] nodes, int[][] graph, int width, int height) {
//        List<Node> neighbors = new ArrayList<>();
//        int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Movements in the order left, right, up, down
//
//        for (int[] direction : directions) {
//            int nextX = node.x;
//            int nextY = node.y;
//
//            // Slide until obstacle is met
//            while (nextX + direction[0] >= 0 && nextX + direction[0] < width && nextY + direction[1] >= 0 && nextY + direction[1] < height
//                    && graph[nextX + direction[0]][nextY + direction[1]] != 0) {
//                nextX += direction[0];
//                nextY += direction[1];
//            }
//
//            if (nextX != node.x || nextY != node.y) {
//                neighbors.add(nodes[nextX][nextY]);
//            }
//        }
//
//        return neighbors;
//    }
}


// REFERENCES

// https://docs.oracle.com/javase%2F7%2Fdocs%2Fapi%2F%2F/java/util/Queue.html
// https://docs.oracle.com/javase/8/docs/api/java/lang/StringBuilder.html
// https://www.geeksforgeeks.org/a-search-algorithm/