#include "s21_graph_algorithms.h"
#include <algorithm>
#include <stack>
#include <queue>
#include <limits>

int* GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    std::stack<int> s;
    s.push(start_vertex);
    while (!s.empty()) {
        int src = s.top();
        if (std::count(traversed_vertices.begin(), traversed_vertices.end(), src)) {
            s.pop();
            continue;
        }
        traversed_vertices.push_back(src);
        s.pop();
        for (int dest : graph.Destinations(src)) {
            s.push(dest);
        }
    }
    // for (int i : traversed_vertices) {
    //     std::cout << i;
    // }
    // std::cout << std::endl;
    return traversed_vertices.data();
}

int* GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    std::queue<int> q;
    q.push(start_vertex);
    while (!q.empty()) {
        int src = q.front();
        if (std::find(traversed_vertices.begin(), traversed_vertices.end(), src) == traversed_vertices.end()) {
            traversed_vertices.push_back(src);
            for (int dest : graph.Destinations(src)) {
                q.push(dest);
            }
        }
        q.pop();
    }
    // for (int i : traversed_vertices) {
    //     std::cout << i;
    // }
    // std::cout << std::endl;
    return &traversed_vertices[0];
}

double GraphAlgorithms::GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2) {
    int num_vertices = graph.GetSize();

    std::vector<double> distances(num_vertices, std::numeric_limits<double>::infinity()); // длины кратчайших путей
    distances[vertex1] = 0;

    for (int iteration = 0; iteration < num_vertices; ++iteration) {
        for (int u = 0; u < num_vertices; ++u) { // по вершинам графа
            for (int v : graph.Destinations(u)) { // где есть направленные ребра из u
                double weight = graph.GetEdgeWeight(u, v);

                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                }
            }
        }
    }
    return distances[vertex2];
}

//std::vector<std::vector<double>> GraphAlgorithms::GetShortestPathsBetweenAllVertices(Graph &graph) {
//    // алгоритма Флойда-Уоршелла я взяла разбирать
//}