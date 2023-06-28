#include <vector>
#include <math.h>
#include <iostream>
#include <assert.h>

#ifndef VEC_HPP
#define VEC_HPP

class vec2f {
    public :
        ~ vec2f() {}
        double x = 0;
        double y = 0;
        vec2f () {}
        vec2f (double _x, double _y) {
            x = _x;
            y = _y;
        }
        vec2f operator*(double s) {
            return vec2f(x*s,y*s);
        }
        vec2f operator+(vec2f v2) {
            return vec2f(x+v2.x,y+v2.y);
        }
        void operator+=(vec2f v2) {
            x += v2.x;
            y += v2.y;
        }
        void operator-=(vec2f v2) {
            x -= v2.x;
            y -= v2.y;
        }
        void operator*=(double s) {
            x *= s;
            y *= s;
        }
        vec2f operator-(vec2f v2) {
            return vec2f(x-v2.x,y-v2.y);
        }
        double squareDistance() {
            return x*x+y*y;
        }
        double squareDistance(vec2f v2) {
            double X = x - v2.x;
            double Y = y - v2.y;
            return X*X+Y*Y;
        }
        double distance(vec2f v2) {
            double X = x - v2.x;
            double Y = y - v2.y;
            return sqrt(X*X+Y*Y);
        }
        double size() {
            return sqrt(squareDistance());
        }
        void scale(double a) {
            double s = size();
            if (s != 0) *this *= a/s;
        }
        void normalize() {
            scale(1);
        }
        void clamp(double a) {
            if (squareDistance() > a*a) scale(a);
        }
        double scalaire(vec2f v2) {
            return x*v2.x + y*v2.y;
        }
        void print() {
            printf(" (%f, %f) ", x, y);
        }
};


using vecR2n = std::vector<vec2f>;

template <class T>
class vec_ij {  //vecteurs de R^(n*(n-1)/2) indicés par i,j pour 1<=i<j<=n
                //donc avec a = i-1, b = j-1, alors L[a][b] = L_i,j et 0<=a<b<n
                //homogène à une matrice triangulaire supérieure sans diagonale de dimension n*n
public:
    
    int n;
    std::vector<T> c; //c[0] n'est pas utilisé
    
    ~ vec_ij<T> () {
    }
    
    vec_ij<T> () {}
    
    vec_ij<T> (int N, T neutre) {
        n = N;   
        fill(neutre);
    }
    
    void fill(T neutre) {
        for (int k = 0; k <= n*(n-1)/2 ; k++) {
            c.push_back( neutre );
        }
    }
    
    T * operator[](int a) { //retourne un vecteur v tel que v[b] = L[a][b]. Permet l'écriture L_i,j "normale"
        //hyp : 0 <= a < n
        assert((0 <= a) && (a < n));
        return c.data() + (int)(a*n - a*(a-1)/2 ); 
        //TODO : recheck if this works
    }
    
    vec_ij<T> operator+(vec_ij<T> v2) { //hyp : n = v2.n
        assert(n == v2.n);
        vec_ij<T> p;
        p.n = n;
        for (int i = 0; i < c.size(); i++) {
            p.c.push_back(c[i] + v2.c[i]);
        }
        return p;
    } 
    
    vec_ij<T> operator-(vec_ij<T> v2) { //hyp : n = v2.n
        assert(n == v2.n);
        vec_ij<T> p;
        p.n = n;
        for (int i = 0; i < c.size(); i++) {
            p.c.push_back(c[i] - v2.c[i]);
        }
        return p;
    }
        
    vec_ij<T> operator*(double s) {
        vec_ij<T> p;
        p.n = n;
        for (int i = 0; i < c.size(); i++) {
            p.c.push_back(c[i] * s);
        }
        return p;
    }
    
    void ajoute_profondeur(int N, T neutre) {
        for (int k = 0; k < n ; k++) {
            c.push_back( neutre );
        }
        n ++;
        if ( N > 0) ajoute_profondeur(N-1, neutre);
    }
    
    void projection_orthogonale_positive() {
        for (int i = 0; i < c.size(); i++) {
            if( c[i] < 0 ) c[i] = 0;
        }
    }
    double min() {
        if (c.size() < 1) return -100;
        double m = c[1];
        for (int i = 2; i < c.size(); i++) {
            if( c[i] < m ) m = c[i];
        }
        return m;
    }
    void print() {
            printf("\n");
        for (int i = 0; i < c.size(); i++) {
                printf("[%d] %f , ", i, c[i]);
        }
        printf("\n\n\n");
    }
};

void print_vecR2n(vecR2n a) {
    for (int i = 0; i < a.size(); i++ ) {
        a[i].print();
    }
    printf("\n");
}

template <class T>
void print_vector(std::vector<T> v) {
    for (int i = 0; i < v.size(); i++ ) {
        printf("%d, ", v[i]);
    }
    printf("\n");
}

double produit_scalaire(vecR2n a, vecR2n b) {
    uint n_min = std::min(a.size(), b.size());
    double s = 0;
    for (int i = 0; i < n_min; i ++) {
        s += a[i].scalaire(b[i]);
    }
    return s;
}

#endif
