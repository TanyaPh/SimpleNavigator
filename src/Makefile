CC=g++ -Wall -Werror -Wextra -std=c++17 -g
CHECKFLAGS=-lgtest -lgmock -pthread
GCOVFLAGS = -fprofile-arcs -ftest-coverage
GCOV=--coverage
OS = $(shell uname)

all: gcov_report

test_graph: clean
	# $(CC) $(GCOV) -c graph/test.cpp 
	$(CC) -c graph/s21_graph.cpp graph/test.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test test.o s21_graph.o $(CHECKFLAGS)
	./test

test_graph_algorithms: clean
	# $(CC) $(GCOV) -c graph_algorithms/main.cpp 
	$(CC) -c graph_algorithms/main.cpp graph_algorithms/s21_graph_algorithms.cpp graph/s21_graph.cpp $(CHECKFLAGS)
	$(CC) $(GCOV) -o test main.o s21_graph_algorithms.o s21_graph.o $(CHECKFLAGS)
	./test


clean:
	rm -f *.gcno *.gcda *.info *.out *.gch
	rm -rf ./gcov_rep.out.dSYM
	rm -rf ./report
	rm -rf ./*.o ./*.a
	rm -rf test
	rm -rf .clang-format
