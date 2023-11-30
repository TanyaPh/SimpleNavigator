#include "s21_GraphController.h"

#include <iostream>

GraphController::GraphController() : graph(), algorithms() {}

void GraphController::LoadGraphFromFile(const std::string& filename) {
  if (graph.LoadGraphFromFile(filename)) {
    std::cout << "Graph loaded successfully." << std::endl;
    for (int i = 0; i < graph.GetSize(); ++i) {
      for (int j = 0; j < graph.GetSize(); ++j) {
        std::cout << graph.GetEdgeWeight(i, j) << ' ';
      }
      std::cout << std::endl;
    }
  } else {
    std::cerr << "Failed to load the graph from file." << std::endl;
  }
}

void GraphController::VisualizeGraph(const std::string& dotFilename) {
  graph.ExportGraphToDot(dotFilename);
  std::cout << "Graph exported to DOT format for visualization." << std::endl;
}

void GraphController::BreadthFirstTraversal(int start_vertex) {
  std::cout << "Breadth-First Search:" << std::endl;
  std::vector<int> bfs = algorithms.BreadthFirstSearch(graph, start_vertex);
  for (int vertex : bfs) {
    std::cout << vertex << " ";
  }
  std::cout << std::endl;
}

void GraphController::DepthFirstTraversal(int start_vertex) {
  std::cout << "Depth-First Search:" << std::endl;
  std::vector<int> dfs = algorithms.DepthFirstSearch(graph, start_vertex);
  for (int vertex : dfs) {
    std::cout << vertex << " ";
  }
  std::cout << std::endl;
}

void GraphController::ShortestPathBetweenVertices(int start_vertex,
                                                  int end_vertex) {
  double shortestPath = algorithms.GetShortestPathBetweenVertices(
      graph, start_vertex, end_vertex);
  std::cout << "Shortest Path from " << start_vertex << " to " << end_vertex
            << ": ";
  if (shortestPath == std::numeric_limits<double>::infinity()) {
    std::cout << "No path exists." << std::endl;
  } else {
    std::cout << shortestPath << std::endl;
  }
}

void GraphController::ShortestPathsBetweenAllVertices() {
  std::vector<std::vector<double>> allShortestPaths =
      algorithms.GetShortestPathsBetweenAllVertices(graph);
  std::cout << "Shortest Paths Between All Vertices:" << std::endl;
  for (const auto& row : allShortestPaths) {
    for (double path : row) {
      std::cout << (path == std::numeric_limits<double>::infinity()
                        ? "INF"
                        : std::to_string(path))
                << " ";
    }
    std::cout << std::endl;
  }
}

void GraphController::MinimumSpanningTree() {
  std::vector<std::vector<double>> mst = algorithms.GetLeastSpanningTree(graph);
  std::cout << "Minimum Spanning Tree:" << std::endl;
  for (const auto& row : mst) {
    for (double weight : row) {
      std::cout << weight << " ";
    }
    std::cout << std::endl;
  }
}

void GraphController::SolveTravelingSalesmanProblem() {
  try {
    TsmResult tsm = algorithms.SolveTravelingSalesmanProblem(graph);
    std::cout << "TSM" << std::endl;
    for (auto i : tsm.vertices) {
      std::cout << i << ' ';
    }
    std::cout << std::endl << tsm.distance << std::endl;
  } catch (std::string& err) {
    std::cout << "TSM\n" << err << std::endl;
  }
}
