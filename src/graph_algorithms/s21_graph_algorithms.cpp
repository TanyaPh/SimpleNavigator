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

std::vector<std::vector<double>> GraphAlgorithms::GetLeastSpanningTree(Graph &graph) {
    std::vector<std::vector<double>> MST(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
    std::vector<int> traversed_vertices;
    int src = 0;
    int dest = 0;
    traversed_vertices.push_back(0);
    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = 0; i < graph.GetSize(); i++) {
        for (int cur_src : traversed_vertices) {
            for (int cur_dest : graph.Destinations(cur_src)) {
                if (std::count(traversed_vertices.begin(), traversed_vertices.end(), cur_dest)) continue;

                double cur_distance = graph.GetEdgeWeight(cur_src, cur_dest);
                if (cur_distance != 0 && min_distance > cur_distance) {
                    min_distance = cur_distance;
                    dest = cur_dest;
                    src = cur_src;
                    // std::cout << src << ' ' << dest << " | " << min_distance << std::endl;
                }
            }
        }
        // std::cout << ">> " << src << ' ' << dest << " | " << min_distance << std::endl;
        if (min_distance != std::numeric_limits<double>::infinity()) {
            MST[src][dest] = min_distance;
            MST[dest][src] = min_distance;
            traversed_vertices.push_back(dest);
            min_distance = std::numeric_limits<double>::infinity();
        }
    }
    return MST;
}   // вроде работает но, требует доработки
