#include "GraphController/s21_GraphController.h"

void menu() {
    std::cout << "====================================================================================" << std::endl;
    std::cout << "Меню:" << std::endl;
    std::cout << "1. Считать матрицу с файла" << std::endl;
    std::cout << "2. Вывести граф в .dot файл" << std::endl;
    std::cout << "3. Поиск в ширину в графе от заданной вершины" << std::endl;
    std::cout << "4. Поиск в глубину в графе от заданной вершины" << std::endl;
    std::cout << "5. Кратчайший путь по алгоритму Дейкстры" << std::endl;
    std::cout << "6. Кратчайшие пути между всеми парами вершин в графе по алгоритму Флойда-Уоршелла" << std::endl;
    std::cout << "7. Поиск наименьшего остовного дерева по алгоритму Прима" << std::endl;
    std::cout << "0. Exit" << std::endl;
    std::cout << "====================================================================================" << std::endl;
}

void handleGraphLoading(GraphController &controller) {
    std::string filename;
    std::cout << "Введите название файла для считывания матрицы смежности: ";
    std::cin >> filename;
    controller.LoadGraphFromFile(filename);
}

void handleGraphVisualization(GraphController &controller) {
    std::string dotFilename;
    std::cout << "Введите название .DOT файла: ";
    std::cin >> dotFilename;
    controller.VisualizeGraph(dotFilename);
}

void handleBreadthFirstTraversal(GraphController &controller) {
    int startVertex;
    std::cout << "Введите стартовую вершину: ";
    std::cin >> startVertex;
    controller.BreadthFirstTraversal(startVertex);
}

void handleDepthFirstTraversal(GraphController &controller) {
    int startVertex;
    std::cout << "Введите стартовую вершину: ";
    std::cin >> startVertex;
    controller.DepthFirstTraversal(startVertex);
}

void handleShortestPath(GraphController &controller) {
    int startVertex, endVertex;
    std::cout << "Введите стартовую вершину: ";
    std::cin >> startVertex;
    std::cout << "Введите конечную вершину: ";
    std::cin >> endVertex;
    controller.ShortestPathBetweenVertices(startVertex, endVertex);
}

void handleAllShortestPaths(GraphController &controller) {
    controller.ShortestPathsBetweenAllVertices();
}

void handleMinimumSpanningTree(GraphController &controller) {
    controller.MinimumSpanningTree();
}

int main() {
    GraphController controller;

    int choice;
    do {
        menu();
        std::cout << "Выберите действие (1-7): ";
        std::cin >> choice;

        switch (choice) {
            case 1:
                handleGraphLoading(controller);
                break;
            case 2:
                handleGraphVisualization(controller);
                break;
            case 3:
                handleBreadthFirstTraversal(controller);
                break;
            case 4:
                handleDepthFirstTraversal(controller);
                break;
            case 5:
                handleShortestPath(controller);
                break;
            case 6:
                handleAllShortestPaths(controller);
                break;
            case 7:
                handleMinimumSpanningTree(controller);
                break;
            case 0:
                std::cout << "Выход" << std::endl;
                break;
            default:
                std::cout << "Некорректный ввод" << std::endl;
        }
    } while (choice != 0);
}