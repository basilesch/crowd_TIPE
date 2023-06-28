#ifndef ENV_HPP
#define ENV_HPP

//#include "physics.hpp"
#include <vector>
#include <iostream>
#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <math.h>
#include "physics.hpp"

#define PI 3.141592


float dot_product(vec2f u, vec2f v)
{
    return u.x*v.x + u.y*v.y;
}

class Wall
{
public:
    float epaisseur = 3;
	Wall(vec2f l_p0, vec2f l_v_dir, float l_d, float l_door_pos, float l_door_size)
	{
		p0 = l_p0; //début du mur
		v_dir = l_v_dir; //direction du mur
		v_dir.normalize();
		d = l_d; //longueur mur

		door_pos = l_door_pos;
		door_size = l_door_size;

		float angle = (v_dir.y < 0 ? -1 : 1) *
                        acos(double(dot_product(v_dir, vec2f(1, 0)) / (sqrt(v_dir.x*v_dir.x + v_dir.y*v_dir.y))));

        sprite1.setPosition(p0.x, p0.y+epaisseur);
		sprite1.setSize(sf::Vector2f((door_pos)*d - door_size/2, epaisseur));
		sprite1.setRotation(180/PI*angle);
        sprite1.setFillColor( sf::Color::Black );
        sprite2.setFillColor( sf::Color::Black );

		sf::Vector2f sprite2_pos;
        sprite2_pos.x = p0.x + cos(angle)*(d*door_pos + door_size/2);
        sprite2_pos.y = p0.y + sin(angle)*(d*door_pos + door_size/2);

		sprite2.setPosition(sprite2_pos);
		sprite2.setSize(sf::Vector2f((1-door_pos)*d - door_size/2, epaisseur));
		sprite2.setRotation(180/PI*angle);
	}
	
	bool est_dedans(vec2f p)
	{
	    return sprite1.getGlobalBounds().contains(p.x, p.y)||sprite2.getGlobalBounds().contains(p.x, p.y);
	}

	vec2f vers_point_plus_proche(vec2f p) { 
        //rend le vecteur pointant de p vers le point du mur le plus proche
        //On profite ici du fait que v_dir est unitaire
        float d_proj = (p.x - p0.x)*v_dir.x + (p.y - p0.y)*v_dir.y;
        
        if ( ( (d_proj - door_size) > door_pos*d ) && ( (d_proj - door_size) > door_pos*d ) ) { //le projeté est "dans la porte"
            if ( d_proj < door_pos*d ) { // plus près de p0
                return ( p0 + v_dir*(d_proj - door_size) - p);
            } else { // plus loin de p0
                return ( p0 + v_dir*(d_proj + door_size) - p);
                
            }
            
        }
        return v_dir * d_proj;
    }
	void Render(sf::RenderWindow* l_render)
	{
	    l_render->draw(sprite1);
	    l_render->draw(sprite2);
	}

	vec2f p0;
	vec2f v_dir;
	float d;

    float door_pos;
	float door_size;

	sf::RectangleShape sprite1;
	sf::RectangleShape sprite2;
};

struct Pylon
{
	Pylon (vec2f l_pos, float l_radius)
	{
		pos = l_pos;
		radius = l_radius;

		sprite.setRadius(radius);
        sprite.setFillColor( sf::Color::Black );
		sprite.setPosition(pos.x-radius, pos.y-radius);
	}

	vec2f pos;
	float radius;
	sf::CircleShape sprite;
};

class environment
{
public:
	environment(){}
	~environment(){}

	void Update()
	{

	}
	void Render(sf::RenderWindow* l_render)
	{
        for (int i = 0; i < walls.size(); i++)
        {
            walls[i].Render(l_render);
        }
        for (int i = 0; i < pylons.size(); i++)
        {
            l_render->draw(pylons[i].sprite);
        }
	}



	std::vector<Wall> walls;
	std::vector<Pylon> pylons;
};


typedef std::vector<std::vector<std::vector<float> > > Data;
environment data2env(Data data) {
    environment env;
        
        for (int i = 0; i < data[0].size(); i ++)
        {
            env.walls.push_back(Wall(vec2f(data[0][i][0], data[0][i][1]),
                                      vec2f(data[0][i][2], data[0][i][3]),
                                      data[0][i][4], data[0][i][5], data[0][i][6]));
        }

        for (int i = 0; i < data[1].size(); i ++)
        {
            env.pylons.push_back(Pylon(vec2f(data[1][i][0], data[1][i][1]), data[1][i][2]));
        }
        return env;
}


#endif
