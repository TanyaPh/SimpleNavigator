#include "s21_graph_algorithms.h"
#include <stack>
#include <queue>

int* GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    std::stack<int> s;
    s.push(start_vertex);
    while (!s.empty()) {
        int src = s.top();
        while (std::count(traversed_vertices.begin(), traversed_vertices.end(), src)) {
            s.pop();
            src = s.top();
        }
        traversed_vertices.push_back(src);
        s.pop();
        for (int dest : graph.Destinations(src)) {
            s.push(dest);
        }
    }
    return traversed_vertices.data();
}

int* GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    std::queue<int> q;
    q.push(start_vertex);
    while (!q.empty()) {
        int src = q.front();
        while (std::find(traversed_vertices.begin(), traversed_vertices.end(), src) != traversed_vertices.end()) {
            q.pop();
            src = q.front();
        }
        traversed_vertices.push_back(src);
        q.pop();
        for (int dest : graph.Destinations(src)) {
            q.push(dest);
        }
    }
    return &traversed_vertices[0];
}
