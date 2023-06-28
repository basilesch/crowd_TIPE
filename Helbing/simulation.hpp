#include "environment.hpp"
#include "CsvReader.hpp"
#include "crowd.hpp"

#ifndef SIM_HPP
#define SIM_HPP



class simulation
{
public :
    simulation(int w, int h, int pop, environment envi) : cro(envi, w, h, pop)
    {
            env = envi;
    };
    ~simulation(){}

    void step(float dt)
    {
        cro.step(env, dt);
    }
    
    void Render(sf::RenderWindow* l_window)
    {
        env.Render(l_window);
        cro.Render(l_window);
    }

    environment env;
    Data data;
    crowd cro;
};

#endif
