#include "particle.h"

Particle::Particle(VectorXd _p, VectorXd _v, double _m){
    setPos(_p);
    setSpeed(_v);
    setMass(_m);
    forces = VectorXd::Zero(_p.rows());
}

void Particle::setPos(VectorXd _p){
    p = _p;
}

void Particle::setSpeed(VectorXd _v){
    v = _v;
}

void Particle::setMass(double _m){
    m = _m;
    if (m == 0){
        w = 1.0;
    } else {
        w = 1.0/m;
    }
}

void Particle::setForce(VectorXd _force){
    forces = _force;
}

void Particle::addForce(VectorXd f){
    forces += f;
}

VectorXd Particle::getPos() const {
    return p;
}

VectorXd Particle::getSpeed() const {
    return v;
}

double Particle::getMass() const {
    return m;
}

double Particle::getInvMass() const {
    return w;
}

VectorXd Particle::getForce() const {
    return forces;
}
