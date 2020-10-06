todo: main
main: tsp.cpp
	mpicxx -pthread -o tsp tsp.cpp -fopenmp
clean:
	rm tsp