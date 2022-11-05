#ifndef OBJECT_H
#define OBJECT_H
#include "particle.h"
#include <vector>
#include <memory>
#include <GL/glew.h>
#include <GL/freeglut.h>

typedef std::shared_ptr<Particle> ParticlePtr;

class Object {
    protected:
        double mass{};
        VectorXd pos{}; // Initial position of the object, no need to update it
        VectorXd v{}; // Initial speed of the object, no need to update it
        GLfloat* color{};
    public:
        std::vector<ParticlePtr> vertices{};

        Object(){};
        Object(double x, double y, double z, double m, GLfloat* c, VectorXd _v = VectorXd::Zero(3)){VectorXd _pos(3); _pos << x,y,z; setPos(_pos); setMass(m); setSpeed(v); color=c; };
        virtual ~Object(){};

        void setMass(double m){mass = m;};
        void setPos(VectorXd _pos) {pos = _pos;};
        void setSpeed(VectorXd _v) {v = _v;};
        double getMass() const {return mass;};
        VectorXd getPos() const {return pos;};
        VectorXd getSpeed() const {return v;};

        virtual void draw() const = 0;

};


#endif