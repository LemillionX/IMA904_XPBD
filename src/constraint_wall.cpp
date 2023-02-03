#include "constraint_wall.h"
#include <iostream>
#include <vector>

WallConstraint::WallConstraint(ParticlePtr q, Vector3d _p1, Vector3d _p2, Vector3d _p3, double _h, double alpha, double stiffness) : Constraint(alpha, stiffness, INEQUALITY){
    std::vector<ParticlePtr> _particles = {q};
    p1 = _p1;
    p2 = _p2;
    p3 = _p3;
    h = _h;
    setParticles(_particles);
    update();
}

double WallConstraint::getThickness() const {
    return h;
}

void WallConstraint::setGradient(){
    int n_rows = particles[0]->getPos().rows();
    grad = MatrixXd::Zero(n_rows,1);
    grad.col(0) =  (p2 - p1).cross(p3-p1).normalized();
}

void WallConstraint::update(){
    Vector3d n = (p2 - p1).cross(p3-p1).normalized();
    double _value = (particles[0]->getPos()-p1).dot(n) - h;
    setValue(_value);
    setGradient();
}