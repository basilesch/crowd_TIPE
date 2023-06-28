#include "simulation.hpp"
#include "rng.hpp"


int main()
{ 
    sf::RenderWindow window(sf::VideoMode(900, 900), "TIPE_Crowd");
    
    std::vector<Data> d;
    d.push_back( CsvReader("Envi.txt") );
    
    int population = 100;
    simulation sim = simulation(900, 900, population, data2env(CsvReader("Envi.txt")));
    int step = 0;
    int stepNum = 20;
    
    sf::Clock clock;
    
    
    while (window.isOpen())
    {
        float dt = .007;//std::min(clock.restart().asSeconds(), (float).007);
        
        sim.step(dt);
        
        
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
        window.display();
        
        if (step%stepNum == 0) {
            sf::Image screenshot = window.capture();
            screenshot.saveToFile("./outputImage/step_" + std::to_string(step/stepNum) + ".png");
        }
        step ++;
    }



    return 0;
}
