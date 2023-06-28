
//DECLARATIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef PHY_HPP
#define PHY_HPP
#include <math.h>

class vec2f {
    public :
        float x = 0;
        float y = 0;
        vec2f () {}
        vec2f (float _x, float _y) {
            x = _x;
            y = _y;
        }
        vec2f operator*(float s) {
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
        void operator*=(float s) {
            x *= s;
            y *= s;
        }
        vec2f operator-(vec2f v2) {
            return vec2f(x-v2.x,y-v2.y);
        }
        float squareDistance() {
            return x*x+y*y;
        }
        float squareDistance(vec2f v2) {
            float X = x - v2.x;
            float Y = y - v2.y;
            return X*X+Y*Y;
        }
        float size() {
            return sqrt(squareDistance());
        }
        void scale(float a) {
            *this *= a/size();
        }
        void normalize() {
            scale(1);
        }
        void clamp(float a) {
            if (squareDistance() > a*a) scale(a);
        }
        
};

class phy {
    
    public :
        vec2f pos, vel, acc;
        float angle = 0;
        
        phy();
        phy(vec2f p);
    
        void step(float dt); 
        void limit(float mx, float my, float dx, float dy, bool loop);
        //void draw(vec2f o, ofColor c, float scale);
    
};
void attract( phy* p1, vec2f p2, float s);
void attract( phy* p1, phy* p2, float s, bool reciprocal);
void attract( phy* p, int l, vec2f pos, float s);
void interAttraction(phy* p, int l, float s);



//PHY METHODS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

phy::phy() {
    pos = vec2f();
    vel = vec2f();
    acc = vec2f();
}
phy::phy(vec2f p) {
    pos = p;
    acc = vec2f(0, 0);
    vel = vec2f(0, 0);
}
void phy::step(float dt) {
    pos += vel * dt;
    vel += acc * dt;
    acc = vec2f(0, 0);
}
void phy::limit(float mx, float my, float dx, float dy, bool loop) {
    //hyp : phy can't travel 2 screens in a frame
    if( loop ) {
        if ( pos.x < mx ) pos.x += dx;
        if ( pos.x > mx + dx ) pos.x -= dx;
        if ( pos.y < my ) pos.y += dy;
        if ( pos.y > my + dy ) pos.y -= dy;            
    } else { //bounce
        if ( pos.x < mx ) vel.x *= -1;
        if ( pos.x > mx + dx ) vel.x *= -1;
        if ( pos.y < my ) vel.y *= -1;
        if ( pos.y > my + dy ) vel.y *= -1;
    }
}
/*
void phy::draw(vec2f o, ofColor c, float scale) {
    ofSetColor(c);
    ofDrawCircle(o + pos*scale, scale*5);
}*/

//PHY RELATED FUNCTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void attract( phy *p1, vec2f p2, float s) {
        // a = m1 * G * m1 * m2 / d^2
        float a = s*3e5 / p1->pos.squareDistance(p2); 
        //if (a > 1e4) a = 1e4;
        vec2f f_1_to_2 =  p2 - p1->pos;
        f_1_to_2.scale(a);
        p1->acc += f_1_to_2;    
}
void attract( phy *p1, phy *p2, float s, bool reciprocal) {
        // a = m1 * G * m1 * m2 / d^2
        float a = s*3e5 / p1->pos.squareDistance(p2->pos); 
        if (a > 1e4) a = 1e4;
        vec2f f_1_to_2 =  p2->pos - p1->pos;
        f_1_to_2.scale(a);
        p1->acc += f_1_to_2;
        if (reciprocal) p2->acc -= f_1_to_2;
}
void attract( phy* p, int l, vec2f pos, float s) {
    for(int i = 0; i < l; i++) {
        attract( p+i, pos, s);
    }
}
void interAttraction(phy* p, int l, float s) {
    for(int i = 0; i < l; i++) {
        for (int j = i+1; j < l; j++) {
            attract(p+i, p+j, s, true);
        }
    }
}

#endif

