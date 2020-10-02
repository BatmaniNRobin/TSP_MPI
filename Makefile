todo: main
main: tsp.cpp
	g++ -pthread -o tsp tsp.cpp
clean:
	rm tsp