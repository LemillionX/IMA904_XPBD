#include "constraint_dist.h"
#include <iostream>
#include <vector>

DistConstraint::DistConstraint(ParticlePtr p1, ParticlePtr p2, double _l0, double alpha, double stiffness) :  Constraint(alpha, stiffness, DISTANCE_CONSTRAINT){
    std::vector<ParticlePtr> _particles = {p1,p2};
    l0 = _l0;
    setParticles(_particles);
    double _value = (p1->getPos() - p2->getPos()).norm() - l0;
    setValue(_value);
    setGradient(_particles);
}



double DistConstraint::getRestLength() const {
    return l0;
}

void DistConstraint::setGradient(std::vector<ParticlePtr> _particles){
    ParticlePtr p0 = _particles[0];
    ParticlePtr p1 = _particles[1];
    int n_rows = p0->getPos().rows();
    grad = MatrixXd::Zero(n_rows, 2);
    VectorXd dir =  (p0->getPos() - p1->getPos()).normalized();
    grad.col(0) = dir;
    grad.col(1) = -dir;
}

void DistConstraint::update(){
    double _value = (particles[0]->getPos() - particles[1]->getPos()).norm() - l0;
    setValue(_value);
    setGradient(particles);
}
