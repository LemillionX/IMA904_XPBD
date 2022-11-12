#include "constraint_fixed.h"

FixedConstraint::FixedConstraint(ParticlePtr p, VectorXd p0, double alpha, double stiffness) : Constraint(alpha, stiffness, FIXED_CONSTRAINT){
    std::vector<ParticlePtr> _particles = {p};
    rest_pos = p0;
    setParticles(_particles);
    setValue((p->getPos() - rest_pos).norm());
    setGradient(_particles);
}

void FixedConstraint::setGradient(std::vector<ParticlePtr> _particles){
    int n_rows =  _particles[0]->getPos().rows();
    grad = MatrixXd::Zero(n_rows, 1);
    grad.col(0) = ( _particles[0]->getPos() - rest_pos) .normalized();
}

void FixedConstraint::update(){
    setValue((particles[0]->getPos() - rest_pos).norm());
    setGradient(particles);
}