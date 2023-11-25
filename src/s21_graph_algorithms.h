#ifndef __S21_GRAPH_ALGORITMS_H__
#define __S21_GRAPH_ALGORITMS_H__

#include "graph/s21_graph.h"

class GraphAlgorithms {
  public:
    GraphAlgorithms() = default;
    GraphAlgorithms(const GraphAlgorithms& other) = delete;
    GraphAlgorithms(GraphAlgorithms&& other) = delete;
    ~GraphAlgorithms() = default;

    int* DepthFirstSearch(Graph &graph, int start_vertex);
    int* BreadthFirstSearch(Graph &graph, int start_vertex);

    double GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2);
    std::vector<std::vector<double>> GetShortestPathsBetweenAllVertices(Graph &graph);

    std::vector<std::vector<double>> GetLeastSpanningTree(Graph &graph);
};

#endif
