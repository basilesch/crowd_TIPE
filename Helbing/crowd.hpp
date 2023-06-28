#include "physics.hpp"
#include "environment.hpp"
#include "rng.hpp"
#include <vector>
#include <math.h>
#include "FastMarching.hpp"

#ifndef CRO_HPP
#define CRO_HPP

using namespace std;

class pedestrian {
    public :
        phy p; //une seule vitesse (ie v = w ) -> on ignore (10) 
        vec2f goal;
        int goal_id = 0;
        float desired_velocity = 40; // v^0_alpha, devrait être gaussienne sur la population
        float max_velocity = 1.6*20; //v^max_alpha, = 1.3*desired_velocity
        pedestrian(phy lp, vec2f lgoal) {
            p = lp;
            goal = lgoal;
        }
};

class crowd {
    public:
		sf::CircleShape sprite;
        vector<pedestrian> ped;  
        float inv_relaxation_time = 1/.25; // 1/ tau alpha
        vector<vec2f> goals;
        
        Grille grille; //pour le fast marching
        Grille grille2; //pour le fast marching
        int pas = 1;
        
        void double_crowd(environment env, int w, int h, uint size) {
            
            vec2f centre = vec2f(w/2, h/2);
            vec2f d1 = centre + vec2f(w/3, 0); //départs
            vec2f d2 = centre - vec2f(0, h/3); 
            vec2f o1 = vec2f(50, h/2); //objectifs
            vec2f o2 = vec2f(w/2, h-50); 
            
            grille = Create_Grille_Fast_Marching(pas, (int)(w/pas), (int)(h/pas), env.pylons, env.walls, Pylon(o1, 1));
            grille2 = Create_Grille_Fast_Marching(pas, (int)(w/pas), (int)(h/pas), env.pylons, env.walls, Pylon(o2, 1));
            
            for (uint i = 0; i < size; i++) {
                bool group = (i%2);
                vec2f p = (group ? d1 : d2) + vec2f(((float)rng(1e6))/1e4-50, ((float)rng(1e6))/1e4-50);
                //vec2f( ((float)(rng(w)*1e4))*.5e-4 + .25*w, ((float)(rng(h)*1e4))*.5e-4 + .25*h );
                //vec2f g = vec2f( ((float)(rng(w)*1e4))*1e-4, ((float)(rng(h)*1e4))*1e-4 );
                ped.push_back( pedestrian(phy(p), (group ? o2 : o1)) );
            }
        }
        
        void one_crowd(environment env, int w, int h, uint size) {
            /*
            vec2f o1 = vec2f( 100, 300 );
            goals.push_back( vec2f(423, 300) );
            goals.push_back( vec2f(823, 300) );
            goals.push_back( vec2f(2000, 300) ); */
            
            vec2f o1 =( vec2f(100, 300) );
            vec2f d1 = vec2f(1100, 300); //départs
            
            grille = Create_Grille_Fast_Marching(pas, (int)(w/pas), (int)(h/pas), env.pylons, env.walls, Pylon(o1, 1));
            grille2 = grille;
            
            for (uint i = 0; i < size; i++) {
                vec2f p = d1 + vec2f(((float)rng(1e6))/1e4-50, ((float)rng(1e6))/1e4-50);
                //vec2f( ((float)(rng(w)*1e4))*.5e-4 + .25*w, ((float)(rng(h)*1e4))*.5e-4 + .25*h );
                //vec2f g = vec2f( ((float)(rng(w)*1e4))*1e-4, ((float)(rng(h)*1e4))*1e-4 );
                ped.push_back( pedestrian(phy(p), o1) );
            }
        }
        
        crowd(environment env, int w, int h, uint size) {
            
            //grille = Create_Grille_Fast_Marching(1, (int)(w/1), (int)(h/1), env.pylons, env.walls, Pylon(vec2f(100.1, 300.1), 1));
            
			sprite.setRadius(4);
            sprite.setFillColor(sf::Color::Red);
            one_crowd(env, w,h,size);
        }
        
        float repulsion = .15;
        
        void step(environment env, float dt) {
            for (int i = 0; i < ped.size(); i++) {
                pedestrian * pie =  & ped[i];               
                
                
                Vitesse v = Calcule_Vitesse( ((i%2) ? grille : grille2), pie->p.pos.x ,pie->p.pos.y, pas);
                vec2f desired_direction = vec2f(v.x, v.y)*100; //(pie->goal - pie->p.pos); //vecteur e alpha (1)
                
                //vec2f desired_direction = (pie->goal - pie->p.pos); //vecteur e alpha (1)
                
                pie->p.acc += ( desired_direction * pie->desired_velocity - pie->p.vel ) * inv_relaxation_time; // (2)                
                
                //with pedestrian
                for (int j = i+1; j < ped.size(); j++) {  // Newtonian instead of (3) and (4) 
                    attract(&pie->p, &ped[j].p, -6*repulsion*.9, true); // *=2
                }
                
                //With walls
                //float a, b, c ; //équation du mur ax + by + c =0
                //float dms, dps, dos, ads; //distance carrées : au mur, à la porte, à l'origine du mur, au mur/porte ajustée pour la répulsion
                for (int j = 0; j < env.walls.size();  j++) { // Newtonian instead of (5)
                    Wall w = (env.walls[j]);
                    
                    float dp = w.d/(int)(w.d/2);//(stop)/((int)stop*3);
                    float dplus =  w.d*w.door_pos - w.door_size/2 -dp;
                    float dminus =  w.d*w.door_pos + w.door_size/2 +dp;
                    //printf("%f %f ", stop, dp);
                    float d_seg_min = 10000000000;
                    float p_min = 0;
                    for (float p = 0; p < w.d + .01; p += dp) {
                        //printf("%f : %f , %f \n", p , dpos - w.door_size, dpos + w.door_size);
                        if(p < dplus || p > dminus) {
                            if (pie->p.pos.squareDistance( w.p0 + w.v_dir*p ) < d_seg_min ) {
                                d_seg_min = pie->p.pos.squareDistance( w.p0 + w.v_dir*p );
                                p_min = p;
                            }
                        }
                    }
                    attract(&pie->p, w.p0 + w.v_dir*p_min , -15*repulsion );
                } 
                
                //with pylons
                for (int j = 0; j < env.pylons.size(); j++) {
                    vec2f diff =  env.pylons[j].pos - pie->p.pos - vec2f(1,2);
                    float s = diff.squareDistance(); 
                    s = (s -(1 + env.pylons[j].radius)*env.pylons[j].radius);
                    s = -repulsion*8e8 / (s*s) ; 
                    diff.scale(s);
                    pie->p.acc += diff;    
                    //attract(&pie->p, env.pylons[j].pos, -env.pylons[j].radius*env.pylons[j].radius*repulsion*1.4 );
                    
                }
                
                //attractive things eventually : TODO (6) et (7)
                //and summarize (3, 4, 5, 6, 7) in (8) and (9)
                //pie->p.acc.clamp(100);                
                pie->p.step(dt);
                if (pie->p.vel.squareDistance() >= pie->max_velocity*pie->max_velocity ) { 
                    pie->p.vel.scale(pie->max_velocity); // (11) et (12)
                }
                
                
                
                //GOALS CHECK
                if (pie->p.pos.squareDistance(pie->goal) < 1800 ) {
                    if (pie->goal_id +1 < goals.size() ) {
                        pie->goal_id = (pie->goal_id + 1)% goals.size();
                        pie->goal = goals[pie->goal_id];
                    }
                }
            }
        }
        
        float cout() { //Ici, la moyenne du carré des distances entre le piéton et l'objectif +1e6* nombre d'objectifs restants.
            float s = 0;
            for (int i = 0; i < ped.size(); i++ ) {
                s += ped[i].p.pos.squareDistance(ped[i].goal) + (goals.size()-1-ped[i].goal_id)*1e6;
            }
            return (s/(ped.size())); 
        }    
		void Render(sf::RenderWindow * rwin) {
			/*sprite.setRadius(38.7);
            sprite.setFillColor(sf::Color(40, 40, 100));
            for (int i = 0; i < goals.size(); i++) {
				sprite.setPosition(goals[i].x-38.7, goals[i].y-38.7);
				rwin->draw(sprite);
            }
            */
			sprite.setRadius(3.3);
			for (int i = 0; i < ped.size(); i++) {
                //sprite.setFillColor( sf::Color(255*(goals.size() - ped[i].goal_id)/goals.size() , 165*(ped[i].goal_id)/goals.size(), 0, 10 ) );
                int num = whash(0xc0ff33 + i);
                sprite.setFillColor( sf::Color(num%256, (num/256)%256, (num/65536)%256) );
				sprite.setPosition(ped[i].p.pos.x, ped[i].p.pos.y);
				rwin->draw(sprite);
            }
            /*
            sf::RectangleShape cache;
            cache.setFillColor( sf::Color::Green ); //(200,255,200));
            cache.setPosition(0, 0);
            cache.setSize(sf::Vector2f(220, 600));
            rwin->draw(cache);
            */            
		}
};

#endif
