CC=g++ -Wall -Werror -Wextra -std=c++17 -g
CHECKFLAGS=-lgtest -lgmock -pthread
GCOVFLAGS = -fprofile-arcs -ftest-coverage
GCOV=--coverage
OS = $(shell uname)

all: gcov_report

test_graph: clean
	$(CC) -c graph/s21_graph.cpp graph/test.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test test.o s21_graph.o $(CHECKFLAGS)
	./test

test_graph_algorithms: clean
	$(CC) -c graph_algorithms/main.cpp graph_algorithms/s21_graph_algorithms.cpp graph/s21_graph.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test main.o s21_graph_algorithms.o s21_graph.o $(CHECKFLAGS)
	./test

test_all: clean
	$(CC) -c main.cpp graph_algorithms/s21_graph_algorithms.cpp graph/s21_graph.cpp GraphController/s21_GraphController.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test main.o s21_graph_algorithms.o s21_graph.o s21_GraphController.o $(CHECKFLAGS)
	./test

test: clean
	$(CC) -c tests.cpp graph_algorithms/s21_graph_algorithms.cpp graph/s21_graph.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test tests.o  s21_graph_algorithms.o s21_graph.o $(CHECKFLAGS)
	./test
	rm -rf test

s21_graph.a:
	$(CC) -c graph/s21_graph.cpp -o s21_graph.o
	@ar rcs s21_graph.a s21_graph.o
	ranlib s21_graph.a

s21_graph_algorithms.a:
	$(CC) -c graph_algorithms/s21_graph_algorithms.cpp -o s21_graph_algorithms.o
	@ar rcs s21_graph_algorithms.a s21_graph_algorithms.o
	ranlib s21_graph_algorithms.a
 
clean:
	rm -f *.gcno *.gcda *.info *.out *.gch
	rm -rf ./gcov_rep.out.dSYM
	rm -rf ./report
	rm -rf ./*.o ./*.a
	rm -rf test
	rm -rf .clang-format
