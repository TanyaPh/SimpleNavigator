#include "s21_graph_algorithms.h"
#include "../graph/s21_graph.h"

int main() {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph_.txt");

    GraphAlgorithms solver;
    int* bfs = solver.BreadthFirstSearch(graph, 0);

    std::cout << "\nBFS" << std::endl;
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

//    std::vector<std::vector<double>> shortestPaths = solver.GetShortestPathsBetweenAllVertices(graph);
//    std::cout << "Кратчайший путь по алгоритму Флойда-Уоршелла:" << std::endl;
//    for (int i = 0; i < graph.GetSize(); ++i) {
//        for (int j = 0; j < graph.GetSize(); ++j) {
//            std::cout << shortestPaths[i][j] << " ";
//        }
//        std::cout << std::endl;
//    }

    auto mst = solver.GetLeastSpanningTree(graph);
    std::cout << "\nMST" << std::endl;
    for (size_t i = 0; i < mst.size(); i++) {
        for (size_t j = 0; j < mst[i].size(); j++) {
            std::cout << mst[i][j] << ' ';
        }
        std::cout << std::endl;
    }

    return 0;
}
