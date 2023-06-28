#include "environment.hpp"
#include "rng.hpp"
#include "vecteurs.hpp"
#include "matrices.hpp"
#include <vector>
#include <math.h>

#include <fstream>
#include <string>
#include "FastMarching.hpp"

#ifndef CRO_HPP
#define CRO_HPP



class crowd {
public:
    sf::CircleShape sprite; //texture de cercle pour le rendu graphique
    
    //Variables sur toute la population
    
    uint n;
    vecR2n q; //Vecteur positions
    std::vector<float> r; //Vecteur rayons
    vecR2n u; //Vecteur vitesses
    vecR2n U; //Vecteur U(q), vitesses désirées: fonction de q
    matrice<double> D; //Vecteur  D_i,j : fonction de q dans R
    std::vector<bool> en_contact; //indicatrice des elements en contact avec d'autres
    environment env;
    
    std::vector<float> pressure;
    
    //Variables sur la population ayant des contacts
    
    uint nc; //nombre de piétons ayant au moins un contact
    uint nct; //nombre d'éléments ayant au moins un contact (y compris les obstacles)
    std::vector<uint> I; // tableau d'indice des elements en contact avec d'autres ; sert d'extractrice
    std::vector<uint> W; // tableau d'indice des murs présentant des contacts ; sert d'extractrice
    std::vector<uint> P; // tableau d'indice des colonnes présentant des contacts ; sert d'extractrice
    matrice<double> Dc; //Vecteur  Dc_i,j : fonction de qc dans R
    matrice<vec2f> e; //Vecteur e_i,j : fonction de qc dans R²
    //permet d'avoir G_i,j (q) = grad(D_i,j) (q) avec (G_i,j)[i] = -e_i,j ; (G_i,j)[j] = e_i,j ; (G_i,j)[k] = 0 si k != i ou j.
    vecR2n Uc; //Vecteur Uc(qc), vitesses désirées: fonction de qc
    vecR2n uc; //Vecteur vitesses 
    
    float max_speed = 100; //en pixel/s
    vec2f goal = vec2f(450, 800);
    vec2f origine = vec2f( 450, 100 );
    vec2f goal2 = vec2f(100, 450);
    vec2f origine2 = vec2f( 450, 100 );//vec2f( 800, 450 );
    
    int width;
    int height;
    std::vector<std::vector<float>> densite;
    
    //pour le fast marching
    Grille grille1;
    Grille grille2;
    
    crowd(environment envi, int w, int h, uint size) { 
        env = envi;
        
        
        grille1 = Create_Grille_Fast_Marching(1, (int)(w/1), (int)(h/1), env.pylons, Pylon(goal, 1));
        grille2 = grille1;
        //grille2 = Create_Grille_Fast_Marching(1, (int)(w/1), (int)(h/1), env.pylons, Pylon(goal2, 1));
            
        generate_crowd(env,w,h,size + env.pylons.size());

        std::vector<float> zero;
        zero.assign(h, 0);
        densite.assign(w, zero); 
        width = w;
        height = h;
    }
        
    void generate_crowd(environment env, int w, int h, uint size) {
        n = size;
        
        int sn = sqrt(n)+1;
        int rmax = 3;
        for (int i = 0; i < n; i++) {
            //vec2f pos = origine + vec2f( 4*rmax * (i%sn), 4*rmax * (i/sn) ) + vec2f(((float)rng(rmax*100))/1e2, ((float)rng(rmax*100))/1e2); //blue noise (compartimenté)
            
            float nb_or = 0.618033988749894842;
            vec2f pos = (i%2) ? origine : origine2;
            pos += vec2f(cos(3 * i * nb_or), 2*sin( 3 * i * nb_or)) * .8 * (rmax/2) * (1 + 5*sqrt(i));  
            
            q.push_back(pos);
            r.push_back(rmax); //push_back(2+ rng((rmax-2)*100)/100.); //entre 2 et rmax
            pressure.push_back(0);
        }
        for (int i = 0; i < env.pylons.size() ; i++) {
            q[i] = env.pylons[i].pos;
        }
        D = matrice<double>(n, 0);
        vecR2n zero;
        zero.assign(n, vec2f(0, 0));
        en_contact.assign(n, false);
        U = zero;
        u = zero;
    }
    
    void step(environment env, float dt, int pas) {
        for (int i = env.pylons.size(); i < n; i++) { //SKIP 0
            int x = (width  + (int)q[i].x)%width ;
            int y = (height + (int)q[i].y)%height;
            //densite[x][y] += (10 + pressure[i] * 255 / pressure_max)*dt;
            
            q[i] += u[i] * dt;
        }
        precalcul_distances_contacts(env);
        calcule_vitesses_desirees( pas);
        
        
        pressure.assign(n, 0);
        u = U;
        if (nc != 0) {
            uc.clear();
            uc = uzawa(dt);
            for (int i = 0; i < nc; i++) {
                if (I[i] >= env.pylons.size()) {//SKIP 0
                    pressure[I[i]] = (uc[i] - u[I[i]]).size();
                    u[I[i]] = uc[i];
                    u[I[i]].clamp(max_speed); //Limitation de la vitesse
                }
            }
        }
    }
    
    
    vecR2n uzawa (float dt) { //Applique l'algorithme d'Uzawa spécifique à cette modélisation
        uint nb_iter_max = 1000; //1000
        float rho = (nct-nc)*1000 + 1000; //50
        float epsilon = .1;//0.01; //seuil de chevauchement acceptable
        
        
//        std::ofstream output_file;
        // std::ios::app is the open mode "append" meaning
        // new data will be written to the end of the file.
        
//        output_file.open("gnuplot/pop_" + std::to_string(n) + "_rho_" + std::to_string(rho) + ".dat" , std::ios::app);
        
        
        vecR2n v;
        v = Uc; //v.assign(nc, vec2f(0, 0))
        matrice<double> mu = phi( Uc, dt ); //matrice<double>(nc, 0); //mu_0
        int nb_iter = 0;
        double d_min = - 2 * epsilon;
        double d_min0 = (Dc - phi(v,dt)).min(); 
        std::cout << " d_min0 = " << d_min0 << " | ";
        
        while ( (nb_iter < nb_iter_max)  && (d_min < -epsilon) ) {
           
            // Ajoute au fichier .dat
//            output_file << nb_iter << "\t" << std::abs (d_min) << std::endl;
            
            vecR2n phi_star_mu = phi_star(mu, dt); 
            for (int i = 0; i < nc; i++ ) {
                if (I[i] < env.pylons.size()) {
                    v[i] = vec2f(0,0); //v_k+1
                    
                } else {
                    v[i] = Uc[i] - phi_star_mu[i]; //v_k+1
                }
                
                //v[i].clamp(10000); //Limitation de la vitesse
            }
            
            matrice<double> phi_v = phi(v, dt);
            mu = mu + ( phi_v - Dc )*rho;
            mu.projection_orthogonale_positive(); //mu_k+1
            
            if ((nb_iter % 20) == 0 ) {
                d_min = (Dc - phi_v).min(); //ne pas vérifier à chaque tour de boucle économise au long terme
                
            }
            //d_min = -(Dc - phi_v).dist_coeff_pos_carre();
            nb_iter ++;
        }
        
        
        std::cout <<  "nb_iter = " << nb_iter
                  << ", d_min = " << d_min
                  << ", nc = " << nc
                  << ", dt = " << dt << std::endl;
        return v;
    }
    
    void precalcul_distances_contacts (environment env) { //calcule D, e, en_contact et I
        I.clear();
        Uc.clear();
        P.clear();
        W.clear();
        
        float epsilon = .2; //range at which the contact is taken into account
        
        std::vector<bool> en_contact, en_contact_p, en_contact_w; 
        en_contact.assign(n, false);
        en_contact_p.assign(env.walls.size(), false);
        en_contact_w.assign(env.pylons.size(), false);
        
        for (int i = 0; i < n; i++ ) {
            for (int j = i+1; j < n; j++ ) {
                D[i][j] = q[i].distance(q[j]) - r[i] - r[j] - ( (i<env.pylons.size()) ? env.pylons[i].radius : 0 ) ;
                if(D[i][j] <= epsilon) {
                    en_contact[i] = true;
                    en_contact[j] = true;
                }
            } 
            for (int j = 0; j < env.walls.size(); j++) {
                //TODO : optimiser en stoquant les valeurs pour les réutiliser
                //std::cout << "square distance" << ((env.walls[j]).vers_point_plus_proche(q[i]).squareDistance()) << std::endl;
                if ( (env.walls[j]).vers_point_plus_proche(q[i]).squareDistance() <= (epsilon + r[i]+env.walls[j].epaisseur)*(epsilon + r[i]+env.walls[j].epaisseur) ) {
                    en_contact[i] = true;
                    en_contact_w[j] = true;
                }
            }/*
            for (int j = 0; j < env.pylons.size(); j++) {
                //TODO : optimiser en stoquant les valeurs pour les réutiliser
                if ( (env.pylons[j].pos - q[i]).squareDistance() <= 
                    (epsilon*60 + env.pylons[j].radius + r[i])*(epsilon*60 + env.pylons[j].radius + r[i])) {
                    en_contact[i] = true;
                    en_contact_p[j] = true;
                }
            }*/
        }
        for (int i = 0; i < n; i++ ) {
            if(en_contact[i]) {
                I.push_back(i);
                Uc.push_back(U[i]);
            }
        }
        for (int i = 0; i < env.walls.size(); i++ ) {
            if(en_contact_w[i]) {
                W.push_back(i);
            }
        }
        for (int i = 0; i < env.pylons.size(); i++ ) {
            if(en_contact_p[i]) {
                P.push_back(i);
            }
        }
        nc = I.size();
        uint nw = W.size() ; //nombre de mur présentant des contacts
        nct = nc + nw + P.size(); //nombre total d'élément avec des contacts
        Dc.c.clear();
        Dc = matrice<double>(nct, 0);
        e.c.clear();
        e = matrice<vec2f>(nct, vec2f(0,0));
        
        for (int i = 0; i < nc; i++ ) { //pour i>nc, Dc et sont nuls
            for (int j = i+1; j < nc; j++ ) {
                Dc[i][j] = D[I[i]][I[j]];
                e[i][j] = (q[I[j]] - q[I[i]]); 
                e[i][j].normalize(); //vecteur de i vers j unitaire
            }
            for (int j = 0; j < nw; j++ ) {
                e[i][j + nc] = (env.walls[W[j]]).vers_point_plus_proche(q[I[i]]) ;
                Dc[i][j + nc] = e[i][j + nc].size() - env.walls[W[j]].epaisseur - r[I[i]];
                e[i][j + nc].normalize();
            }
            for (int j = 0; j < P.size(); j++ ) {
                e[i][j + nc + nw] = env.pylons[P[j]].pos - q[I[i]];
                Dc[i][j + nc + nw] = e[i][j + nc + nw].size() - env.pylons[P[j]].radius - r[I[i]];
                e[i][j + nc + nw].normalize();
            }
        }
    }
    
    void calcule_vitesses_desirees (int pas) { //calcule U
    
        //version basique : se dirige vers g avec vitesse maximale de max_speed pixel/s
        
        for (int i = 0; i < n; i++) {
            
            Vitesse v = (i%2) ? Calcule_Vitesse(grille1, q[i].x, q[i].y, pas) : Calcule_Vitesse(grille2, q[i].x, q[i].y, pas);
            U[i] = vec2f(v.x, v.y);
            if (U[i].squareDistance() < .000001) {
                U[i] = (goal - q[i]);
            }
            U[i].scale(max_speed);
        }
    } 
    
    matrice<double> phi (vecR2n v, float dt) { //Modélisation des contraintes : fonction de v
        matrice<double> res = matrice<double>(nc, 0);
        for (int i = 0; i < nc; i++ ) {
            for (int j = i+1; j < nc; j++ ) { //entre piétons
                res[i][j] = -dt * (-e[i][j].scalaire(v[i]) + e[i][j].scalaire(v[j])); // = produit_scalaire( G[i][j], v );
            }
            for (int j = 0; j < W.size(); j++ ) { //avec les murs
                res[i][j + nc] = dt * e[i][j + nc].scalaire(v[j]); 
                // = -dt * produit_scalaire( G[i][j_mur], v );
            }
            for (int j = 0; j < P.size(); j++ ) { //avec les colonnes
                res[i][nct - 1 - j] = dt * e[i][nct - 1 - j].scalaire(v[j]); 
                // = -dt * produit_scalaire( G[i][j_colonne], v );
            }
        }
        return res;
        
    }
    
    vecR2n phi_star (matrice<double> lambda, float dt) { //Solution du lagrangien : fonction de lambda
        vecR2n res;
        //res.assign(nc, vec2f(0, 0));
        for (int i = 0; i < nc; i++) {
            res.push_back( vec2f(0, 0) );
        }
        
        for (int i = 0; i < nc; i++ ) { // entre piétons
            for (int j = i+1; j < nc; j++ ) {
                res[i] += ( e[i][j] * lambda[i][j] * dt);
                res[j] -= ( e[i][j] * lambda[i][j] * dt);
            }
            for (int j = 0; j < W.size(); j++ ) { //avec les murs
                res[i] +=  e[i][j + nc] * dt * lambda[i][j + nc];
            }
            for (int j = 0; j < P.size(); j++ ) { //avec les colonnes
                res[i] +=  e[i][nct - 1 - j] * dt * lambda[i][nct - 1 - j];
            }
        }
        
        return res;
    }
    
    float pressure_max = 100;
    
    void Render(sf::RenderWindow * rwin) {
        //sprite.setFillColor(sf::Color::Red);
        
        //sprite.setOutlineColor(sf::Color(200,0,0));
        //sprite.setOutlineThickness(1);
        for (int i = env.pylons.size(); i < n; i++) {
            float red = pressure[i] * 255 / pressure_max;
            red = (red > 255) ? 255 : red;
            if (true) {
                sprite.setFillColor(sf::Color(red, 255 - red, 0));
            } else {
                sprite.setFillColor(sf::Color(red, 0, 255 - red));
            }
            sprite.setRadius(r[i]);
            sprite.setPosition(q[i].x, q[i].y);
            rwin->draw(sprite);
        }
        /*
        sprite.setFillColor(sf::Color(0,150,0,20));
        sprite.setOutlineColor(sf::Color(0,100,0));
        sprite.setRadius(30);
        sprite.setPosition(goal.x - 15, goal.y - 15);
        rwin->draw(sprite); 
        sprite.setPosition(goal2.x - 15, goal2.y - 15);
        rwin->draw(sprite); 
        */
        /*
        sprite.setOutlineColor(sf::Color(200,0,0));
        sprite.setOutlineThickness(0);
        sprite.setRadius(1.01);
        /*
        for (int i = 0; i < n; i++) {
            int x = (width  + (int)q[i].x)%width ;
            int y = (height + (int)q[i].y)%height;
            densite[x][y] += (10 + pressure[i] * 255 / pressure_max);
            float red = densite[x][y];
            red = (red > 255) ? 255 : red;
            sprite.setFillColor(sf::Color(red, 255 - red, 0));
            sprite.setPosition(x, y);
            rwin->draw(sprite);
        }*/
        
    }
};

#endif
