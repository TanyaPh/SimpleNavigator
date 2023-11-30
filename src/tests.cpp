#include <gtest/gtest.h>
#include <filesystem>

#include "graph/s21_graph.h"
#include "s21_graph_algorithms.h"

TEST(GraphTest, LoadGraph) {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph10.txt");
    std::vector<std::vector<int>> expectation = {{0,   29,  20,  21,  16,  31,  100, 12,  4 ,  31,  18},
                                                 {29,  0,   15,  29,  28,  40,  72,  21,  29,  41,  12},
                                                 {20,  15,  0 ,  15,  14,  25,  81,  9 ,  23,  27,  13},
                                                 {21,  29,  15,  0 ,  4 ,  12,  92,  12,  25,  13,  25},
                                                 {16,  28,  14,  4 ,  0 ,  16,  94,  9 ,  20,  16,  22},
                                                 {31,  40,  25,  12,  16,  0 ,  95,  24,  36,  3 ,  37},
                                                 {100, 72,  81,  92,  94,  95,  0 ,  90,  101, 99,  84},
                                                 {12,  21,  9 ,  12,  9 ,  24,  90,  0 ,  15,  25,  13},
                                                 {4,   29,  23,  25,  20,  36,  101, 15,  0 ,  35,  18},
                                                 {31,  41,  27,  13,  16,  3 ,  99,  25,  35,  0 ,  38},
                                                 {18,  12,  13,  25,  22,  37,  84,  13,  18,  38,  0}};
    EXPECT_EQ(expectation.size(), graph.GetSize());
    for (auto i = 0; i < expectation.size(); i++) {
        std::vector<int> result = graph.Destinations(expectation[i][0]);
        for (auto j = 1; j < expectation[i].size(); j++) {
            EXPECT_EQ(expectation[i], result[i]);
        }
    }
}

// TEST(GraphTest, ExportGraph) {
//     Graph graph;
//     graph.ExportGraphToDot("examples/output.dot");
// }

TEST(GraphTest, Destinations) {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph10.txt");
    std::vector<int> expectation = {20, 15, 0, 15, 14, 25, 81, 9, 23, 27, 13};
    std::vector<int> result = graph.Destinations(2);
    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphTest, GetSize) {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph10.txt");
    EXPECT_EQ(11, graph.GetSize());
}

TEST(GraphTest, GetEdgeWeight) {
    Graph graph;
    graph.LoadGraphFromFile("examples/graph10.txt");
    EXPECT_EQ(12, graph.GetEdgeWeight(4, 5));
}

////////////////////////////////////////////////////

TEST(GraphAlgorithmsTest, BFS) {
    Graph graph;
    GraphAlgorithms solver;    
    graph.LoadGraphFromFile("examples/graph5.txt");

    std::vector<int> expectation = {0, 1, 2, 3, 4};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 0);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphAlgorithmsTest, BFS_start_3) {
    Graph graph;
    GraphAlgorithms solver;    
    graph.LoadGraphFromFile("examples/graph4.txt");

    std::vector<int> expectation = {3, 0, 2, 1, 4};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 3);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphAlgorithmsTest, BFS_tree) {
    Graph graph;
    GraphAlgorithms solver;    
    graph.LoadGraphFromFile("examples/graph_tree.txt");

    std::vector<int> expectation = {0, 1, 2, 3, 4, 5, 6};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 0);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphAlgorithmsTest, DFS) {
    Graph graph;
    GraphAlgorithms solver;    
    graph.LoadGraphFromFile("examples/graph5.txt");

    std::vector<int> expectation = {0, 2, 4, 3, 1};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 0);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphAlgorithmsTest, DFS_start_3) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph4.txt");

    std::vector<int> expectation = {3, 2, 4, 0, 1};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 3);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

TEST(GraphAlgorithmsTest, DFS_tree) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph_tree.txt");

    std::vector<int> expectation = {0, 2, 6, 5, 1, 4, 3};
    std::vector<int> result = solver.BreadthFirstSearch(graph, 0);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        EXPECT_EQ(expectation[i], result[i]);
    }
}

// TEST(GraphAlgorithmsTest, ShortestPathBetweenVertices) {
//     Graph graph;
//     GraphAlgorithms solver;
//     graph.LoadGraphFromFile("examples/graph4.txt");

//     std::vector<int> expectation = {3, 2, 4, 0, 1};
//     std::vector<int> result = solver.BreadthFirstSearch(graph, 3);

//     EXPECT_EQ(expectation.size(), result.size());
//     for (auto i = 0; i < expectation.size(); i++) {
//         EXPECT_EQ(expectation[i], result[i]);
//     }
// }

// TEST(GraphAlgorithmsTest, ShortestPathsBetweenAllVertices) {
//     Graph graph;
//     GraphAlgorithms solver;
//     graph.LoadGraphFromFile("examples/graph4.txt");

//     std::vector<int> expectation = {3, 2, 4, 0, 1};
//     std::vector<int> result = solver.BreadthFirstSearch(graph, 3);

//     EXPECT_EQ(expectation.size(), result.size());
//     for (auto i = 0; i < expectation.size(); i++) {
//         EXPECT_EQ(expectation[i], result[i]);
//     }
// }

TEST(GraphAlgorithmsTest, MST) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph4.txt");

    std::vector<std::vector<double>> expectation = {{0, 1, 1, 1, 0},
                                                    {1, 0, 0, 0, 0},
                                                    {1, 0, 0, 0, 1},
                                                    {1, 0, 0, 0, 0},
                                                    {0, 0, 1, 0, 0}};
    std::vector<std::vector<double>> result = solver.GetLeastSpanningTree(graph);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        for (auto j = 1; j < expectation[i].size(); j++) {
            EXPECT_EQ(expectation[i], result[i]);
        }
    }
}

TEST(GraphAlgorithmsTest, MST_tree) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph_tree.txt");

    std::vector<std::vector<double>> expectation = {{0, 1, 1, 0, 0, 0, 0},
                                                    {1, 0, 0, 1, 1, 0, 0},
                                                    {1, 0, 0, 0, 0, 1, 1},
                                                    {0, 1, 0, 0, 0, 0, 0},
                                                    {0, 1, 0, 0, 0, 0, 0},
                                                    {0, 0, 1, 0, 0, 0, 0},
                                                    {0, 0, 1, 0, 0, 0, 0}};
    std::vector<std::vector<double>> result = solver.GetLeastSpanningTree(graph);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        for (auto j = 1; j < expectation[i].size(); j++) {
            EXPECT_EQ(expectation[i], result[i]);
        }
    }
}

TEST(GraphAlgorithmsTest, MST_value) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph_mst.txt");

    std::vector<std::vector<double>> expectation = {{0, 4, 0, 0, 0, 0, 0, 8, 0},
                                                    {4, 0, 0, 0, 0, 0, 0, 0, 0},
                                                    {0, 0, 0, 7, 0, 4, 0, 0, 2},
                                                    {0, 0, 7, 0, 9, 0, 0, 0, 0},
                                                    {0, 0, 0, 9, 0, 0, 0, 0, 0},
                                                    {0, 0, 4, 0, 0, 0, 2, 0, 0},
                                                    {0, 0, 0, 0, 0, 2, 0, 1, 0},
                                                    {8, 0, 0, 0, 0, 0, 1, 0, 0},
                                                    {0, 0, 2, 0, 0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> result = solver.GetLeastSpanningTree(graph);

    EXPECT_EQ(expectation.size(), result.size());
    for (auto i = 0; i < expectation.size(); i++) {
        for (auto j = 1; j < expectation[i].size(); j++) {
            EXPECT_EQ(expectation[i], result[i]);
        }
    }
}

TEST(GraphAlgorithmsTest, TSM) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph10.txt");

    TsmResult result = solver.SolveTravelingSalesmanProblem(graph);
    EXPECT_EQ(253, result.distance);
}

TEST(GraphAlgorithmsTest, TSM_5) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph5.txt");

    TsmResult result = solver.SolveTravelingSalesmanProblem(graph);
    EXPECT_EQ(5, result.distance);
}

TEST(GraphAlgorithmsTest, TSM_exception_1) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph4.txt");

    ASSERT_THROW(solver.SolveTravelingSalesmanProblem(graph));
}

TEST(GraphAlgorithmsTest, TSM_exception_tree) {
    Graph graph;
    GraphAlgorithms solver;
    graph.LoadGraphFromFile("examples/graph_tree.txt");

    ASSERT_THROW(solver.SolveTravelingSalesmanProblem(graph));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
