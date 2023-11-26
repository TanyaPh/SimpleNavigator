#ifndef __S21_GRAPH_H__
#define __S21_GRAPH_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class Graph {
public:
    Graph();
    ~Graph();
    bool LoadGraphFromFile(const std::string& filename);
    void ExportGraphToDot(const std::string& filename);
    std::vector<int> Destinations(int src);
    int GetSize() const;
    double GetEdgeWeight(int source, int destination) const;

private:
    int size; // размер матрицы смежности
    std::vector<std::vector<int>> adjacencyMatrix; // матрица смежности

private:
    bool IsValidVertex(int vertex) const;
};

#endif // S21_GRAPH_H
