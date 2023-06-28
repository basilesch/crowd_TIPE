#include <iostream>
#include <vector>
#include <math.h>
#include <limits>
#include <utility>

#include "vecteurs.hpp"
#include "environment.hpp"
#include "rng.hpp"

#ifndef FMC_HPP
#define FMC_HPP


float INF = std::numeric_limits<float>::max();

struct Vitesse
{
    Vitesse()
    {
        x = 0; y  = 0;
    }

    Vitesse(float l_x, int l_y)
    {
        x = l_x;
        y = l_y;
    }

    float x;
    float y;
};

enum State
{
    Eclaire, Penombre, Ombre
};

struct Cercle
{
    float r;
    float x;
    float y;
};

struct Point
{
    Point(State s, int l_i, int l_j, int l_x, int l_y, float l_D)
    {
        state = s; i = l_i; j = l_j; y = l_y; D = l_D; x = l_x;
    }

    State state;
    int i;
    int j;
    int x;
    int y;
    float D;
};

typedef std::vector<std::vector<Point> > Grille;


int floor_bis(float x)
{
    int res = floor(x);
    return res == x ? res-1 : res;
}

int ceil_bis(float x)
{
    int res = ceil(x);
    return res == x ? res+1 : res;
}

float Distance(float x1,float y1, float x2, float y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

bool Dans_Pylon(Pylon& p, int x, int y)
{
    return Distance(x, y, p.pos.x, p.pos.y) <= p.radius;
}

float mini(float a, float b)
{
    /*Renvoie le minimum entre a et b*/
    return a > b ? b : a;
}

void normalize(Vitesse &vit, float a)
{
    //Normalize la vitesse vit
    float l = sqrt(vit.x*vit.x + vit.y*vit.y);
    vit.x /= l*1/a;
    vit.y /= l*1/a;
}

void Affiche_grille(const Grille& g)
{
    //Affiche la grille g
    for (int i = 0; i < g.size(); i++)
    {
        for (int j = 0; j < g[i].size(); j++)
        {
            Point p = g[i][j];
            if (p.D == INF)
            {
                std::cout << "I";
            }
            else
            {
                std::cout << p.D;
            }
            std::cout << "  ";
        }
        std::cout << std::endl;
    }
}

void Ajoute_penombre(std::vector<Point>& p_penombres, Point p)
{
    /*Ajoute p a p_penombres en faisant en sorte que p_penombres reste croissant en fonction de D
    Hyp : p_penombres est croissant en fonction de D*/

    for (int i = 0; i < p_penombres.size(); i++)
    {
        if (p_penombres[i].D >= p.D)
        {
            p_penombres.insert(p_penombres.begin() + i, p);
            return;
        }
    }
    p_penombres.push_back(p);
}

void Calcule_D(Grille& g, int i, int j, int pas)
{
    /*Calcule la valeur de g[i][j]*/
    int t_y = g.size(); // nombre de lignes de g
    int t_x = g[0].size(); // nombre de colonnes de g

    float a;
    if (i-1 < 0)
        a = g[i+1][j].D;
    else if (i + 1 >= t_y)
        a = g[i-1][j].D;
    else
        a = mini(g[i-1][j].D, g[i+1][j].D);

    float b;
    if(j - 1 < 0)
        b = g[i][j+1].D;
    else if(j + 1 >= t_x)
        b = g[i][j-1].D;
    else
        b = mini(g[i][j+1].D, g[i][j-1].D);

    if (a == INF || b == INF || abs(a - b) >= pas)
    {
        g[i][j].D = pas + mini(a, b);
        return;
    }
    g[i][j].D = (a + b + sqrt(2*pas*pas - pow(a-b, 2)))/2;

}

std::vector<std::pair<int, int> > Get_Voisins(int i, int j, int t_x, int t_y)
{
    /*Renvoie la liste des voisins valides de la case i j, c'est-a-dire les indices des voisins qui sont encore dans
    les intervalles [0, t_x[x[0, t_y[*/
    std::vector<std::pair<int, int> > voisins;
    for (int di = -1; di != 2; di++)
    {
        for (int dj = -1; dj != 2; dj++)
        {
            int vi = i + di;
            int vj = j + dj;
            if (vi < t_y && vi >= 0 && vj < t_x && vj >= 0 && (dj + di == 1 || dj + di == -1))
            {
                std::pair<int, int> v (vi, vj);
                voisins.push_back(v);
            }
        }
    }
    return voisins;
}

std::vector<Point> Init_Grille(Grille& g, int pas, int t_x, int t_y, std::vector<Pylon> pylons, Pylon sortie)
{
    /*Remplie la grille selon les conditions initiales de l'algorithme de Fast Marching
    Cette grille aura t_x colonnes et t_y lignes
    Renvoie les points dans la penombre a la fin de l'initialisation*/
    std::vector<Point> p_eclaires;

    for (int i = 0; i < t_y; i++)
    {
        std::vector<Point> n_l;

        for (int j = 0; j < t_x; j++)
        {
            Point n_p(Ombre, i, j, j*pas, i*pas, INF);
            for(int a = 0; a < pylons.size(); a++)
            {
                if (Dans_Pylon(pylons[a], n_p.x, n_p.y))
                {
                    n_p.D = INF;
                    n_p.state = Eclaire;
                    p_eclaires.push_back(n_p);
                    break;
                }
            }
            if (Dans_Pylon(sortie, n_p.x, n_p.y))
            {
                n_p.D = 0;
                n_p.state = Eclaire;
                p_eclaires.push_back(n_p);
                //std::cout << "Est dans la sortie x = " << n_p.x << " , y = " << n_p.y << std::endl;
            }
            n_l.push_back(n_p);
        }
        g.push_back(n_l);
    }

    std::vector<Point> p_penombres;

    for (int i = 0; i < p_eclaires.size(); i++)
    {
        Point p = p_eclaires[i];
        if (g[p.i][p.j].D != INF)
        {
            std::vector< std::pair<int, int> > vois = Get_Voisins(p.i, p.j, t_x, t_y);

            for (int k = 0; k < vois.size(); k++)
            {
                int vi = vois[k].first;
                int vj = vois[k].second;
                if (g[vi][vj].state == Ombre)
                {
                    g[vi][vj].state = Penombre;
                    Calcule_D(g, vi, vj, pas);
                    Ajoute_penombre(p_penombres,(g[vi][vj]));
                }
            }
        }
    }

    return p_penombres;
}

void Fast_Marching (Grille& g, std::vector<Point>& p_penombres, int pas)
{
    /*Calcule les coefficients de g selon l'algorithme Fast Marching
    p_penombres est la liste des etats initialement dans la penombre, range par D croissant
    Pas est le pas de la grille*/
    int t_y = g.size(); // Nombre de lignes de la grille
    int t_x = g[0].size(); // Nombre de colonnes de la grille

    while (p_penombres.size() > 0)
    {
        Point p = p_penombres[0]; //Point en cours de traitement, compte tenu de la construction de p_penombres
                                  //ce point est celui avec le plus petit poids D
        int min_i = p.i;
        int min_j = p.j;

        g[min_i][min_j].state = Eclaire;

        p_penombres.erase(p_penombres.begin());

        std::vector< std::pair<int, int> > vois = Get_Voisins(p.i, p.j, t_x, t_y);

        for (int k = 0; k < vois.size(); k++)
        {
            int vi = vois[k].first;
            int vj = vois[k].second;
            if (g[vi][vj].state == Ombre)
            {
                g[vi][vj].state = Penombre;
                Calcule_D(g, vi, vj, pas);
                Ajoute_penombre(p_penombres,(g[vi][vj]));
            }
        }
    }
}

float taux_accroiss (float d1, float d2, float pas)
{
    /*Calcule le taux d'accroissement entre d1 et d2 separe de pas*/
    if (d1 == INF && d2 == INF)
    {
        return 0;
    }
    if (d1 == INF)
    {
        return INF;
    }
    if (d2 == INF)
    {
        return -INF;
    }    
    return ((d1 - d2) / pas);
}

Vitesse Calcule_Vitesse(Grille& g, float x, float y, int pas)
{
    /*Calcule la vitesse souhaitee localement au point de coordonnees x y
    Hyp : 0 <= y <= g.size * pas et 0 <= x <= g[0].size * pas*/
    int i_moins = floor(y / pas);
    int i_plus = ceil(y / pas);
    int j_moins = floor(x / pas);
    int j_plus = ceil(x / pas);

    float gradX_up = taux_accroiss(g[i_moins][j_moins].D,g[i_moins][j_plus].D, pas);
    float gradX_down = taux_accroiss(g[i_plus][j_moins].D, g[i_plus][j_plus].D, pas);
    float gradY_left = taux_accroiss(g[i_moins][j_moins].D, g[i_plus][j_moins].D, pas);
    float gradY_right = taux_accroiss(g[i_moins][j_plus].D, g[i_plus][j_plus].D, pas);

    float coeff_X = x - (pas*j_moins);
    float coeff_Y = y - (pas*i_moins);

    Vitesse v;
    v.x = gradX_down*coeff_Y/pas + gradX_up*(1-coeff_Y)/pas;
    v.y = gradY_right*coeff_X/pas + gradY_left*(1-coeff_X)/pas;
    if ( v.x * v.x + v.y * v.y > 100000) {
        return Vitesse(0,0); //Vitesse((rng(1000)-500)/10,(rng(1000)-500)/10);
    }
    return v;
}

void Ajouter_Obs(std::vector<Cercle>& obs_l, float x, float y, float r)
{
    Cercle new_cercle;
    new_cercle.x = x;
    new_cercle.y = y;
    new_cercle.r = r;

    obs_l.push_back(new_cercle);
}

Grille Create_Grille_Fast_Marching(int pas, int t_x, int t_y, std::vector<Pylon> pylons, Pylon sortie)
{
    Grille g;

    std::vector<Point> p_penombres = Init_Grille(g, pas, t_x, t_y, pylons, sortie);
    Fast_Marching(g, p_penombres, pas);

    return g;
}

#endif
