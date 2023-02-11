#include "constraint.h"
#include <iostream>


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

void Constraint::setGradient(){
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
    update();
    if (type != FIXED_CONSTRAINT){
        if (abs(value) > 1e-08){
            if (type == EQUALITY || (type == INEQUALITY && value < 1e-06)){
                double alpha_t = alpha/(dt*dt);
                double num = -value - alpha_t *lambda;
                double denom = alpha_t;
                double mass_sum = 0;
                // Need to assert grad.cols() == particles.size()
                for (int i = 0; i < grad.cols(); i++){
                    if (alpha < 1e-06){
                        denom += grad.col(i).norm()*grad.col(i).norm();
                    } else {
                        denom += particles[i]->getInvMass()*grad.col(i).norm()*grad.col(i).norm();
                    }
                    mass_sum += particles[i]->getInvMass();
                }

                // Updating lambda and x
                // Don't forget to store the particle at the GLOBAL solving before
                double deltaLambda = 0.0;
                if (denom > 0.001){
                    deltaLambda = num/denom;
                }

                for (size_t i = 0; i < particles.size(); i ++){
                    VectorXd x = particles[i]->getPos();
                    VectorXd deltaX = particles[i]->getInvMass()*grad.col(i)*deltaLambda;
                    if (alpha < 1e-06 && denom > 0.001){ // PBD case, we suppose particles haven't individual masses for simplicity
                        deltaX = particles[i]->getInvMass()*(-value/denom)*grad.col(i)/mass_sum;
                        // if (abs(num) > denom){
                        //     std::cout << "deltaX = " << deltaX.transpose() << std::endl;
                        //     std::cout << "value = " << value << std::endl;
                        //     std::cout << "denom = " << denom << std::endl;
                        //     std::cout << "grad = \n " << grad << std::endl;
                        //     for (auto & p : particles){
                        //         std::cout << "pos = " << p->getPos().transpose() << std::endl;
                        //         std::cout << "speed = " << p->getSpeed().transpose() << std::endl;
                        //         std::cout << "force = " << p->getForce().transpose() << std::endl;
                        //         std::cout << "inv_mass = " << p->getInvMass() << std::endl;
                        //     }
                        //     std::cin.get();
                        // }
                    }
                    particles[i]->setPos(x + deltaX);
                }

                setLagrangeMultiplier(lambda + deltaLambda);
            }
        }
    }


    update();


}