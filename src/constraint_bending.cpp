#include "constraint_bending.h"
#include <iostream>
#include <vector>

BendingConstraint::BendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double _phi0, double alpha, double stiffness) : Constraint(alpha, stiffness, DISTANCE_CONSTRAINT){
    std::vector<ParticlePtr> _particles = {p1, p2, p3, p4};
    phi0 = _phi0;
    setParticles(_particles);
    update();
}


double BendingConstraint::getDihedralAngle() const{
    return phi0;
}

void setGradient() {
    
}

void BendingConstraint::update(){
    VectorXd n1 = (particles[1]->getPos()-particles[0]->getPos()).cross(particles[2]->getPos() - particles[0]->getPos());
    n1 = n1.normalized();
    VectorXd n2 = (particles[1]->getPos()-particles[0]->getPos()).cross(particles[3]->getPos()-particles[0]->getPos());
    n2 = n2.normalized();
    double _value = acos(n1.dot(n2)) - phi0;
    setValue(_value);
    setGradient();
}