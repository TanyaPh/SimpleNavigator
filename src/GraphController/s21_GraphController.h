#ifndef A2_SIMPLENAVIGATOR_V1_0_2_GRAPHCONTROLLER_H
#define A2_SIMPLENAVIGATOR_V1_0_2_GRAPHCONTROLLER_H
#include <limits>

#include "../graph/s21_graph.h"
#include "../graph_algorithms/s21_graph_algorithms.h"

class GraphController {
 public:
  GraphController();

  void LoadGraphFromFile(const std::string& filename);
  void VisualizeGraph(const std::string& dotFilename);
  void BreadthFirstTraversal(int start_vertex);
  void DepthFirstTraversal(int start_vertex);
  void ShortestPathBetweenVertices(int start_vertex, int end_vertex);
  void ShortestPathsBetweenAllVertices();
  void MinimumSpanningTree();
  void SolveTravelingSalesmanProblem();

 private:
  Graph graph;
  GraphAlgorithms algorithms;
};
#endif  // A2_SIMPLENAVIGATOR_V1_0_2_GRAPHCONTROLLER_H
