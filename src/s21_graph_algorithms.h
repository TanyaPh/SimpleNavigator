#ifndef __S21_GRAPH_ALGORITMS_H__
#define __S21_GRAPH_ALGORITMS_H__

#include "graph/s21_graph.h"

struct TsmResult {
    std::vector<int> vertices;
    double distance;
};
// typedef struct TsmResult TsmResult;

class GraphAlgorithms {
  public:
    GraphAlgorithms() = default;
    GraphAlgorithms(const GraphAlgorithms& other) = delete;
    GraphAlgorithms(GraphAlgorithms&& other) = delete;
    ~GraphAlgorithms() = default;

    std::vector<int> DepthFirstSearch(Graph &graph, int start_vertex);
    std::vector<int> BreadthFirstSearch(Graph &graph, int start_vertex);

    double GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2);
    std::vector<std::vector<double>> GetShortestPathsBetweenAllVertices(Graph &graph);

    std::vector<std::vector<double>> GetLeastSpanningTree(Graph &graph);
    TsmResult SolveTravelingSalesmanProblem(Graph &graph);
  
  private:
    std::vector<double> transitionProbabilities(Graph &graph, std::vector<std::vector<double>> &pheromones,
                                                std::vector<bool> &visited, int src);
    int chooseNextDestination(std::vector<double> &probabilityToVertex);
    void addPheromone(TsmResult &ant_route, std::vector<std::vector<double>> &add_pher);
    void updatePheromones(std::vector<std::vector<double>> &pheromones, std::vector<std::vector<double>> &add_pher);
    TsmResult createRoute(Graph &graph, std::vector<std::vector<double>> &pheromones, int src);
};

#endif
