#include "s21_graph.h"

Graph::Graph() : size(1), adjacencyMatrix(1, std::vector<int>(1, 0)) {}

Graph::~Graph() {
    adjacencyMatrix.clear();
}

bool Graph::IsValidVertex(int vertex) const {
    return (vertex >= 0) && (vertex < size);
}

bool Graph::LoadGraphFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename << std::endl;
        return false;
    }

    adjacencyMatrix.clear();

    file >> size;
    adjacencyMatrix.resize(size, std::vector<int>(size, 0));

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            file >> adjacencyMatrix[i][j];
            std::cout << adjacencyMatrix[i][j] << ' ';
        }
        std::cout << std::endl;
    }

    file.close();
    return true;
}

void Graph::ExportGraphToDot(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Couldn't open file " << filename << std::endl;
        return;
    }
    
    bool isWeighted = false; // для проверки, взвешенный или нет
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[i][j] != 1) {
                isWeighted = true;
                break;
            }
        }
        if (isWeighted) {
            break;
        }
    }

    file << (isWeighted ? "digraph" : "graph") << " graphname {" << std::endl;

    // вершины
    for (int i = 0; i < size; ++i) {
        file << "     " << i << ";" << std::endl;
    }

    // рёбра
    for (int i = 0; i < size; ++i) {
        for (int j = i + 1; j < size; ++j) {
            if (adjacencyMatrix[i][j] != 0) {
                file << "     " << i << (isWeighted ? " -> " : " -- ") << j;
                // if (isWeighted) {
                //     file << " [" << adjacencyMatrix[i][j] << "]";
                // } // если взвешенный, то сюда можно добавить вывод веса, если нужно
                file << ";" << std::endl;
            }
        }
    }

    file << "}" << std::endl;
    file.close();
}


std::vector<int> Graph::Destinations(int src) {
    std::vector<int> dest;
    for (std::vector<int>::size_type i = 0; i < adjacencyMatrix[src].size(); i++) {
        if (adjacencyMatrix[src][i] != 0) 
            dest.push_back(i);
    }
    return dest;
}

int Graph::GetSize() const {
    return size;
}

double Graph::GetEdgeWeight(int source, int destination) const {
    return adjacencyMatrix[source][destination];
}
