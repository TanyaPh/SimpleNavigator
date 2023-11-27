#include "s21_graph_algorithms.h"
// #include "graph/s21_graph.h"

int main() {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph4.txt");

    GraphAlgorithms solver;
    int* bfs = solver.BreadthFirstSearch(graph, 0);

    std::cout << "BFS" << std::endl;
    for (int i = 0; i < graph.GetSize(); i++) {
        std::cout << bfs[i] << ' ';
    }

    int* dfs = solver.DepthFirstSearch(graph, 0);
    std::cout << "\nDFS" << std::endl;
    for (int i = 0; i < graph.GetSize(); i++) {
        std::cout << dfs[i] << ' ';
    }

    auto mst = solver.GetLeastSpanningTree(graph);
    std::cout << "\nMST" << std::endl;
    for (int i = 0; i < mst.size(); i++) {
        for (int j = 0; j < mst[i].size(); j++) {
            std::cout << mst[i][j] << ' ';
        }
        std::cout << std::endl;
    }



    return 0;
}