#include "s21_graph_algorithms.h"
// #include "graph/s21_graph.h"

int main() {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph10.txt");

    GraphAlgorithms solver;
    int* bfs = solver.BreadthFirstSearch(graph, 1);
    int* dfs = solver.DepthFirstSearch(graph, 1);

    std::cout << "BFS" << std::endl;
    for(int i = 0; i < 11; i++) {
        std::cout << bfs[i] << ' ';
    }

    std::cout << "\nDFS" << std::endl;
    for(int i = 0; i < 11; i++) {
        std::cout << bfs[i] << ' ';
    }

    return 0;
}
