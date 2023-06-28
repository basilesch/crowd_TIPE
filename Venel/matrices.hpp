#include <vector>
#include <iostream>

#ifndef MAT_HPP
#define MAT_HPP

template <class T>
class matrice{
public :
    int n;
    std::vector<T> c; 
    
    ~ matrice<T> () {
        c.clear();
    }
    matrice<T> () {}
    matrice<T> (int N, T neutre) {
        n = N;   
        c.clear();
        for (int i = 0; i < n*n; i++) {
            c.push_back(neutre);
        }
        //c.assign(n*n, neutre);
    }
    T * operator[](int a) {
        //hyp : 0 <= a < n
        return c.data() + (int)(a*n); 
    }
    matrice<T> operator+(matrice<T> v2) { //hyp : n = v2.n
        if (n == 0) return (*this);
        matrice<T> p = matrice<T>(n, (*this)[n-1][n-1]);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                p[i][j] = (*this)[i][j] + v2[i][j];
            }
        }
        return p;
    } 
    matrice<T> operator-(matrice<T> v2) { //hyp : n = v2.n
        if (n == 0) return (*this);
        matrice<T> p = matrice<T>(n, (*this)[n-1][n-1]);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                p[i][j] = (*this)[i][j] - v2[i][j];
            }
        }
        return p;
    } 
    matrice<T> operator*(double s) { 
        if (n == 0) return (*this);
        matrice<T> p = matrice<T>(n, (*this)[n-1][n-1]);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                p[i][j] = (*this)[i][j] * s;
            }
        }
        return p;
    } 
    void projection_orthogonale_positive() {
        for (int i = 0; i < c.size(); i++) {
            if( c[i] < 0 ) c[i] = 0;
        }
    }
    double min() {
        if (c.size() == 0) return -100;
        double m = c[0];
        for (int i = 1; i < c.size(); i++) {
            if( c[i] < m ) m = c[i];
        }
        return m;
    }
    double dist_coeff_pos_carre() {
        //calcule le carre de la distance euclidienne à l'ensemble des matrices à coefficients positifs
        float res = 0;
        for (int i = 0; i < c.size(); i++) {
            if( c[i] < 0 ) res += c[i] * c[i];
        }
        return res;
        
    }
    
    void print() {
        printf("n = %d ; \n", n);
        for (int i = 0; i < n; i++) {
            printf("\n%d : ", i );
            for (int j = 0; j < n; j++) {
                printf("%f, ", (*this)[i][j]);
            }
        }
        printf("\n\n");
    }
};

#endif
