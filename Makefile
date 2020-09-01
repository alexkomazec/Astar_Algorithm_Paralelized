lall: Astar

Astar: Astar.cpp
	g++ -fopenmp -Wall -o Astar Astar.cpp

.PHONY: clean

clean:
	rm Astar
