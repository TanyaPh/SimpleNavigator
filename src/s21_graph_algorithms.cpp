#include "s21_graph_algorithms.h"
#include <algorithm>
#include <stack>
#include <queue>
#include <climits>
#include <random>
#include <cmath>


std::vector<int>  GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
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
    return traversed_vertices;
}

std::vector<int>  GraphAlgorithms::BreadthFirstSearch(Graph &graph, int start_vertex) {
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
    return traversed_vertices;
}

std::vector<std::vector<double>> GraphAlgorithms::GetLeastSpanningTree(Graph &graph) {
    std::vector<std::vector<double>> MST(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
    std::vector<int> traversed_vertices;
    int src = 0;
    int dest = 0;
    traversed_vertices.push_back(0);
    double min_distance = INT_MAX;

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
        if (min_distance != INT_MAX) {
            MST[src][dest] = min_distance;
            MST[dest][src] = min_distance;
            traversed_vertices.push_back(dest);
            min_distance = INT_MAX;
        }
    }
    return MST;
}

// std::vector<std::vector<double>> GraphAlgorithms::GetLeastSpanningTree(Graph &graph) {
//     std::vector<std::vector<double>> MST(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
//     std::vector<int> parent(graph.GetSize(), 0);
//     std::vector<int> distance(graph.GetSize(), INT_MAX);
//     std::vector<bool> visited(graph.GetSize(), false);

//     distance[0] = 0;
//     parent[0] = -1;

//     for (int i = 0; i < graph.GetSize() - 1; i++){
//         double min_distance = INT_MAX;
//         int dest;
//         for (int j = 0; j < graph.GetSize(); j++) {
//             if (visited[j] == false && distance[j] < min_distance) {
//                 min_distance = distance[j];
//                 dest = j;
//             }
//         }
//         visited[dest] = true;
//         for (int cur_dest : graph.Destinations(dest)) {
//             if (visited[cur_dest] == false && graph.GetEdgeWeight(dest, cur_dest) < distance[cur_dest]) {
//                 parent[cur_dest] = dest; 
//                 distance[cur_dest] = graph.GetEdgeWeight(dest, cur_dest);
//             }
//         }

//     }

//     return MST;
// }

// int minKey(int key[], bool mstSet[])
// {
//     // Initialize min value
//     int min = INT_MAX, min_index;
 
//     for (int v = 0; v < V; v++)
//         if (mstSet[v] == false && key[v] < min)
//             min = key[v], min_index = v;
 
//     return min_index;
// }
 
// // // A utility function to print the
// // // constructed MST stored in parent[]
// // void printMST(int parent[], int graph[V][V])
// // {
// //     cout << "Edge \tWeight\n";
// //     for (int i = 1; i < V; i++)
// //         cout << parent[i] << " - " << i << " \t"
// //              << graph[i][parent[i]] << " \n";
// // }
 
// // // Function to construct and print MST for
// // // a graph represented using adjacency
// // // matrix representation
// void primMST(int graph[V][V]) {
//     std::vector<std::vector<double>> MST(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
//     int parent[V];
//     int key[V];
//     bool mstSet[V];
    
//     std::vector<int> parent(graph.GetSize(), 0);
//     std::vector<int> distance(graph.GetSize(), INT_MAX);
//     std::vector<bool> visited(graph.GetSize(), false);
 
//     for (int i = 0; i < V; i++)
//         key[i] = INT_MAX, mstSet[i] = false;

//     key[0] = 0;
//     parent[0] = -1;

//     distance[0] = 0;
//     parent[0] = -1;
//     for (int i = 0; i < graph.GetSize() - 1; i++){
//         double min_distance = INT_MAX;
//         int dest;
//         for (int j = 0; j < graph.GetSize(); j++) {
//             if (visited[j] == false && distance[j] < min_distance) {
//                 min_distance = distance[j];
//                 dest = j;
//             }
//         }
//         visited[dest] = true;
//         // for (int j = 0; j < graph.GetSize(); j++) {
//         //     if () {

//         //     }
//         // }
//         for (int cur_dest : graph.Destinations(dest)) {
//             if (visited[cur_dest] == false && graph.GetEdgeWeight(dest, cur_dest) < distance[cur_dest]) {
//                 parent[cur_dest] = dest; 
//                 distance[cur_dest] = graph.GetEdgeWeight(dest, cur_dest);
//             }
//         }

//     }
 
//     // The MST will have V vertices
//     for (int count = 0; count < V - 1; count++) {
//         int min = INT_MAX, min_index;
//         for (int v = 0; v < V; v++)
//             if (mstSet[v] == false && key[v] < min)
//                 min = key[v], min_index = v;


//         mstSet[u] = true;

//         for (int v = 0; v < V; v++) {
//             if (graph[u][v] && mstSet[v] == false
//                 && graph[u][v] < key[v])
//                 parent[v] = u, key[v] = graph[u][v];
//         }
//     }
// }

// std::vector<std::vector<double>> GraphAlgorithms::GetLeastSpanningTree(Graph &graph) {
//     std::vector<std::vector<double>> MST(graph.GetSize(), std::vector<double>(graph.GetSize(), 0));
//     std::priority_queue<std::pair<int,int>, std::vector<std::pair<int,int>>, std::greater<std::pair<int,int>>> pq;
//     std::vector<bool> visited(graph.GetSize(), false);
//     int src = 0;
//     pq.push({0, 0});

//     while (!pq.empty()) {
//         int distance = pq.top().first;  // Weight of the edge
//         int dest = pq.top().second;  // Vertex connected to the edge
//         pq.pop();
         
//         if (visited[dest] == true) continue;  // Skip if the vertex is already visited
//         MST[src][dest] = distance;
//         MST[dest][src] = distance;
//         visited[dest] = true;  // Mark the vertex as visited
        
//         src = dest;
//         for (int dest : graph.Destinations(src)) {
//             if(visited[dest] == false) {
//                 pq.push({graph.GetEdgeWeight(src, dest), dest});  // Add the adjacent edge to the priority queue
//             }
//         }
//     }
//     return MST;
// }

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
        if (cur_ant.vertices.size() != graph.GetSize() + 1) continue;
        addPheromone(cur_ant, add_pher);
        if (cur_ant.distance < result.distance) {
            result = cur_ant;
            i = 0;
        }
        if (i % ants == 0)
            updatePheromones(pheromones, add_pher);
    }
    std::cout << graph.GetSize() + 1 << std::endl;
    if (result.vertices.size() != graph.GetSize() + 1 || result.distance == INT_MAX)
        // std::cout << "error " << std::endl;
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
        // std::cout << res.vertices.size() << std::endl;
    }
    if (graph.GetEdgeWeight(src, res.vertices[0]) != 0){
        res.vertices.push_back(res.vertices[0]);
        res.distance += graph.GetEdgeWeight(src, res.vertices[0]);
    }
    if (res.vertices.size() != graph.GetSize() + 1) 
        res.distance = INT_MAX;

    // std::cout << "\nRES" << std::endl;
    // for (auto i : res.vertices) {
    //     std::cout << i << ' ';
    // }
    // std::cout << std::endl;
    // std::cout << "dis: " << res.distance << std::endl;
    return res;
}

std::vector<double> GraphAlgorithms::transitionProbabilities(Graph &graph, std::vector<std::vector<double>> &pheromones,
                                                            std::vector<bool> &visited, int src) {
    std::vector<double> probabilityToVertex(graph.GetSize(), 0);
    double alfa = 12;
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

int GraphAlgorithms::chooseNextDestination(std::vector<double> &probabilityToVertex) { // need remake
    double choose = (double)rand() / RAND_MAX;
    for (size_t dest = 0; dest < probabilityToVertex.size(); dest++) {
        if (probabilityToVertex[dest] > choose) {
            return dest;
        }
    }
    return -1;
}

void GraphAlgorithms::addPheromone(TsmResult &ant_route, std::vector<std::vector<double>> &add_pher) {
    int q = 12;
    double delta = q / ant_route.distance;
    int src = ant_route.vertices[0];
    for (auto i = 1; i < ant_route.vertices.size(); i++) {
        add_pher[src][ant_route.vertices[i]] += delta;
        add_pher[ant_route.vertices[i]][src] += delta;
    }
}

void GraphAlgorithms::updatePheromones(std::vector<std::vector<double>> &pheromones, std::vector<std::vector<double>> &add_pher) {
    double p = 0.8;
    for (int i = 0; i < pheromones.size(); i++) {
        for (int j = 0; j < pheromones[i].size(); j++) {
            pheromones[i][j] = pheromones[i][j] * p + add_pher[i][j];
            add_pher[i][j] = 0;
        }
    }
}
