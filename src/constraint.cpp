#include "constraint.h"

Constraint::Constraint(double _alpha, double _stiffness, constraint_type _type){
    setCompliance(_alpha);
    setStiffness(_stiffness);
    type = _type;
}


Constraint::Constraint(std::vector<ParticlePtr> _particles, double _alpha, double _stiffness, constraint_type _type){
    setParticles(_particles);
    setCompliance(_alpha);
    setStiffness(_stiffness);
    type = _type;
}

void Constraint::setValue(double v){
    value = v;
}

void Constraint::setStiffness(double s){
    stiffness = s;
}

void Constraint::setCompliance(double a){
    alpha = a;
}

void Constraint::setLagrangeMultiplier(double l){
    lambda = l;
}

void Constraint::setParticles(std::vector<ParticlePtr> _particles){
    particles = _particles;
}

void Constraint::setGradient(std::vector<ParticlePtr> particles){
    int n_rows = particles[0]->getPos().rows();
    size_t n_cols = particles.size();
    grad = MatrixXd::Zero(n_rows, n_cols);

}

constraint_type Constraint::getType() const {
    return type;
}

double Constraint::getValue() const {
    return value;
}

double Constraint::getStiffness() const {
    return stiffness;
}

double Constraint::getCompliance() const {
    return alpha;
}

double Constraint::getLagrangeMultiplier() const {
    return lambda;
}

std::vector<ParticlePtr> Constraint::getParticles() const {
    return particles;
}

MatrixXd Constraint::getGradient() const {
    return grad;
}

void Constraint::solveConstraint(double dt) {
    double alpha_t = alpha/(dt*dt);
    double num = -value - alpha_t *lambda;
    double denom = alpha_t;

    // Need to assert grad.cols() == particles.size()
    for (int i = 0; i < grad.cols(); i++){
        denom += particles[i]->getInvMass()*grad.col(i).norm()*grad.col(i).norm();
    }

    // Updating lambda and x
    // Don't forget to store the particle at the GLOBAL solving before
    double deltaLambda = num/denom;
    for (size_t i = 0; i < particles.size(); i ++){
        VectorXd x = particles[i]->getPos();
        VectorXd deltaX = particles[i]->getInvMass()*grad.col(i)*deltaLambda;
        particles[i]->setPos(x + deltaX);
    }

    setLagrangeMultiplier(lambda + deltaLambda);

    update();


}