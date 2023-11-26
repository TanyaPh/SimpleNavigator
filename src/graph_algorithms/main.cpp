#include "s21_graph_algorithms.h"
#include "../graph/s21_graph.h"

int main() {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph_.txt");

    GraphAlgorithms solver;
    int* bfs = solver.BreadthFirstSearch(graph, 0);

    std::cout << "BFS" << std::endl;
    for(int i = 0; i < graph.GetSize(); i++) {
        std::cout << bfs[i] << ' ';
    }

    int* dfs = solver.DepthFirstSearch(graph, 0);
    std::cout << "\nDFS" << std::endl;
    for(int i = 0; i < graph.GetSize(); i++) {
        std::cout << dfs[i] << ' ';
    }

    int start = 0, end = 4;
    double shortestPath = solver.GetShortestPathBetweenVertices(graph, start, end);
    std::cout << "\nПо алгоритму Дейкстры расстояние от " << start << " до " << end << ": " << shortestPath << std::endl;


    return 0;
}
