#ifndef __S21_GRAPH_ALGORITMS_H__
#define __S21_GRAPH_ALGORITMS_H__


#include <iostream>
#include <vector>

class GraphAlgorithms {
//  private:
 public:
  GraphAlgorithms() = default;
  GraphAlgorithms(const GraphAlgorithms& other) = delete;
  GraphAlgorithms(GraphAlgorithms&& other) = delete;
  ~GraphAlgorithms() = default;
  int* DepthFirstSearch(Graph &graph, int start_vertex);
  int* BreadthFirstSearch(Graph &graph, int start_vertex);

};

#endif
