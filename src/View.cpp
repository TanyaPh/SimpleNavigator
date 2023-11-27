#include <iostream>

int main() {
    std::string input;
    std::cout << "Enter path to load file with grath\n >> ";
    std::cin >> input;
    Graph graph;
    graph.LoadGraphFromFile(input); 
    std::cout << "Choose point and enter number:\n1) BFS\n 
    2) DFS\n3) the shortest path between two vertices\n
    4) the shortest paths between all pairs of vertices\n
    5) the minimal spanning tree\n
    6) the salesman problem\n
    7) chande graph\n>> ";
    std::cin >> input;
    GraphAlgorithms solver;
    switch (input) {
    case 1:
        std::cout << "Enter start vertex\n >> ";
        std::cin >> input;
        solver.BreadthFirstSearch(graph, input);
        break;
    case 2:
        std::cout << "Enter start vertex\n >> ";
        std::cin >> input;
        solver.DepthFirstSearch(graph, input);
        break;
    case 3:
        std::cout << "Enter start and finish vertecies\n >> ";
        std::cin >> vertex1;
        std::cin >> vertex2;
        solver.GetShortestPathBetweenVertices(graph, vertex1, vertex2);
        break;
    case 4:
        solver.GetShortestPathsBetweenAllVertices(graph);
        break;
    case 5:
        solver.GetLeastSpanningTree(graph);
        break;
    case 6:
        solver.SolveTravelingSalesmanProblem(graph);
        break;
    case 7:
        std::cout << "Enter path to load file with grath\n >> ";
        std::cin >> input;
        graph.LoadGraphFromFile(input);
        break;
    
    default:
        std::cout << "Incorrect input, try again\n >> ";
        std::cin >> input;
        break;
    }

    return 0;
}