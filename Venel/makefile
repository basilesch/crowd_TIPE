run: crowd
	./crowd

cleano:
	rm *.o


matrices.o: matrices.hpp
	g++ -c matrices.hpp -o matrices.o
	
vecteurs.o: vecteurs.hpp
	g++ -c vecteurs.hpp -o vecteurs.o
	
crowd_venel.o: crowd_venel.hpp 
	g++ -c crowd_venel.hpp -o crowd_venel.o

simulation.o: simulation.hpp
	g++ -c simulation.hpp -o simulation.o
	
FastMarching.o: FastMarching.hpp
	g++ -c FastMarching.hpp -o FastMarching.o

crowd: main.cpp simulation.o crowd_venel.o vecteurs.o matrices.o FastMarching.o
	g++ main.cpp -lsfml-graphics -lsfml-window -lsfml-system   -o crowd
	chmod 755 crowd

prime : main_prime.cpp simulation.o  matrices.o crowd_venel.o vecteurs.o
	g++ main_prime.cpp -o prime
	chmod 755 prime
	
log: crowd
	./crowd > log.txt
