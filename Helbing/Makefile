cleano:
	rm *.o

genetic.o: genetic.hpp
	g++ -c genetic.hpp -o genetic.o

CsvReader.o: CsvReader.hpp
	g++ -c CsvReader.hpp -o CsvReader.o

environment.o: environment.hpp
	g++ -c environment.hpp -o environment.o

crowd.o: crowd.hpp
	g++ -c crowd.hpp -o crowd.o

simulation.o: simulation.hpp
	g++ -c simulation.hpp -o simulation.o
	
FastMarching.o: FastMarching.hpp
	g++ -c FastMarching.hpp -o FastMarching.o

crowd: main.cpp simulation.o environment.o CsvReader.o crowd.o genetic.o FastMarching.o
	g++ main.cpp -lsfml-graphics -lsfml-window -lsfml-system   -o crowd
	chmod 755 crowd

run: crowd
	./crowd

log: crowd
	./crowd > log.txt
