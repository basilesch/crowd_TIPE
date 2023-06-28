#include "simulation.hpp"
#include "rng.hpp"
#include "genetic.hpp"

int main()
{ 
    //wyhash64_x = 0x892738; //0x178268; //seed
    sf::RenderWindow window(sf::VideoMode(1200, 600), "TIPE_Crowd");
    
    std::vector<data> d;
    d.push_back( CsvReader("Envi.txt") );
    for (int i = 1; i < 1; i++) {
        d.push_back(d[0]);
    }
    
    //population pop = population(1200, 600, 400, d );
    int pop = 400;
    simulation sim = simulation(1200, 600, pop, data2env(CsvReader("Envi.txt")));
    int step = 0;
    int stepNum = 50;
    
    sf::Clock clock;
    //window.clear(sf::Color::White);

    while (window.isOpen())
    {
        float dt = .02; //clock.restart().asSeconds(); //std::min(clock.restart().asSeconds(), (float).01);
        
        sim.step(dt);
        //pop.step(dt);
        
        sf::Event event;
        
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }
        
       window.clear(sf::Color::White);

        sim.Render(&window);
        //pop.pop[0].Render(&window);

        window.display(); //*/
        
        if (step%stepNum == 0) {
            sf::Image screenshot = window.capture();
            screenshot.saveToFile("./outputImage/step_" + std::to_string(step/stepNum) + ".png");
        }
        step ++;
        
        
        //printf("step = %d ; dt = %f \n ; generation = %d ; time left = %f ", step, dt, pop.generation, pop.time_left );
    }



    return 0;
}
