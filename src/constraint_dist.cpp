#include "constraint_dist.h"
#include <iostream>
#include <vector>

DistConstraint::DistConstraint(ParticlePtr p1, ParticlePtr p2, double _l0, double alpha, double stiffness) :  Constraint(alpha, stiffness, EQUALITY){
    std::vector<ParticlePtr> _particles = {p1,p2};
    l0 = _l0;
    setParticles(_particles);
    update();
}



double DistConstraint::getRestLength() const {
    return l0;
}

void DistConstraint::setGradient(){
    int n_rows = particles[0]->getPos().rows();
    grad = MatrixXd::Zero(n_rows, 2);
    VectorXd dir =  (particles[0]->getPos() - particles[1]->getPos()).normalized();
    grad.col(0) = dir;
    grad.col(1) = -dir;
}

void DistConstraint::update(){
    double _value = (particles[0]->getPos() - particles[1]->getPos()).norm() - l0;
    setValue(_value);
    setGradient();
}
