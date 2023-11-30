#include "s21_graph_algorithms.h"
#include "Stack/s21_stack.hpp"
#include "Queue/s21_queue.hpp"
#include <algorithm>
#include <stack>
#include <queue>
#include <limits>
#include <cmath>

std::vector<int> GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    s21::Stack<int> s;
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
    return traversed_vertices;
}

std::vector<int> GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex) {
    std::vector<int> traversed_vertices;
    s21::Queue<int> q;
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
    return traversed_vertices;
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

// алгоритм Флойда-Уоршелла
std::vector<std::vector<double>> GraphAlgorithms::GetShortestPathsBetweenAllVertices(Graph &graph) {
    int num_vertices = graph.GetSize();
    std::vector<std::vector<double>> shortest_paths(num_vertices, std::vector<double>(num_vertices, std::numeric_limits<double>::infinity()));

    for (int u = 0; u < num_vertices; ++u) {
        for (int v : graph.Destinations(u)) {
            double weight = graph.GetEdgeWeight(u, v);
            shortest_paths[u][v] = weight; // начальные значения
        }
    }

    for (int k = 0; k < num_vertices; ++k) {
        for (int i = 0; i < num_vertices; ++i) {
            for (int j = 0; j < num_vertices; ++j) {
                shortest_paths[i][j] = std::min(shortest_paths[i][j], shortest_paths[i][k] + shortest_paths[k][j]);
            }
        }
    }

    return shortest_paths;
}

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
                }
            }
        }
        if (min_distance != std::numeric_limits<double>::infinity()) {
            MST[src][dest] = min_distance;
            MST[dest][src] = min_distance;
            traversed_vertices.push_back(dest);
            min_distance = std::numeric_limits<double>::infinity();
        }
    }
    return MST;
}

TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(Graph &graph) {
    int ants = graph.GetSize();
    std::vector<std::vector<double>> pheromones(graph.GetSize(), std::vector<double>(graph.GetSize(), 0.2));
    std::vector<std::vector<double>> add_pher(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
    srand(time(0));
    int src = 0;
    TsmResult result = createRoute(graph, pheromones, src);
    addPheromone(result, add_pher);
    for (auto i = 1; i < 2400; i++) {
        int src = i % ants;
        TsmResult cur_ant = createRoute(graph, pheromones, src);
        if ((int)cur_ant.vertices.size() != graph.GetSize() + 1) continue;
        addPheromone(cur_ant, add_pher);
        if (cur_ant.distance < result.distance) {
            result = cur_ant;
            i = 0;
        }
        if (i % ants == 0)
            updatePheromones(pheromones, add_pher);
    }
    if ((int)result.vertices.size() != graph.GetSize() + 1 || result.distance == std::numeric_limits<double>::infinity())
        throw std::string("it is impossible to solve");
    return result;
}

TsmResult GraphAlgorithms::createRoute(Graph &graph, std::vector<std::vector<double>> &pheromones, int src) {
    TsmResult res {};
    std::vector<bool> visited(graph.GetSize(), false);
    res.vertices.push_back(src);
    for (auto i = 0; i < graph.GetSize() - 1; i++) {
        visited[src] = true;
        auto probabilityDest = transitionProbabilities(graph, pheromones, visited, src);
        int dest = chooseNextDestination(probabilityDest);
        if (dest == -1) return res;
        res.vertices.push_back(dest);
        res.distance += graph.GetEdgeWeight(src, dest);
        src = dest;
    }
    if (graph.GetEdgeWeight(src, res.vertices[0]) != 0){
        res.vertices.push_back(res.vertices[0]);
        res.distance += graph.GetEdgeWeight(src, res.vertices[0]);
    }
    if ((int)res.vertices.size() != graph.GetSize() + 1) 
        res.distance = std::numeric_limits<double>::infinity();
    return res;
}

std::vector<double> GraphAlgorithms::transitionProbabilities(Graph &graph, std::vector<std::vector<double>> &pheromones,
                                                            std::vector<bool> &visited, int src) {
    std::vector<double> probabilityToVertex(graph.GetSize(), 0);
    double alfa = 1;
    double beta = 4;
    double sum;
    for (int next : graph.Destinations(src)) {
        if (visited[next] == true) continue;
        double t = pow(pheromones[src][next], alfa);
        double n = pow(1 / graph.GetEdgeWeight(src, next), beta);
        probabilityToVertex[next] = t * n;
        sum += probabilityToVertex[next];
    }
    double prev = 0;
    for (auto &i : probabilityToVertex) {
        if (i == 0) continue;
        double p = i / sum;
        i = p + prev;
        prev = i;
    }
    return probabilityToVertex;
}

int GraphAlgorithms::chooseNextDestination(std::vector<double> &probabilityToVertex) {
    double choose = (double)rand() / RAND_MAX;
    for (size_t dest = 0; dest < probabilityToVertex.size(); dest++) {
        if (probabilityToVertex[dest] > choose) 
            return dest;
    }
    return -1;
}

void GraphAlgorithms::addPheromone(TsmResult &ant_route, std::vector<std::vector<double>> &add_pher) {
    int q = 12;
    double delta = q / ant_route.distance;
    int src = ant_route.vertices[0];
    for (size_t i = 1; i < ant_route.vertices.size(); i++) {
        add_pher[src][ant_route.vertices[i]] += delta;
        add_pher[ant_route.vertices[i]][src] += delta;
    }
}

void GraphAlgorithms::updatePheromones(std::vector<std::vector<double>> &pheromones, std::vector<std::vector<double>> &add_pher) {
    double p = 0.8;
    for (size_t i = 0; i < pheromones.size(); i++) {
        for (size_t j = 0; j < pheromones[i].size(); j++) {
            pheromones[i][j] = pheromones[i][j] * p + add_pher[i][j];
            add_pher[i][j] = 0;
        }
    }
}
