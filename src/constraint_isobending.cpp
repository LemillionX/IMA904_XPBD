#include "constraint_isobending.h"
#include <iostream>
#include <vector>

IsobendingConstraint::IsobendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double alpha, double stiffness) : Constraint(alpha, stiffness, EQUALITY){
    std::vector<ParticlePtr> _particles = {p1, p2, p3, p4};
    setParticles(_particles);
    setHessian();
    update();
}


MatrixXd IsobendingConstraint::getHessian() const{
    return Q;
}

double IsobendingConstraint::cotan(Vector3d e1, Vector3d e2) const {
    double dot = e1.dot(e2);
    VectorXd cross = e1.cross(e2);
    if (cross.norm() < 0.000001){
        return 0;
    } else {
        return dot/cross.norm();
    }
}

void IsobendingConstraint::setHessian(){
    Vector3d e0 = particles[1]->getPos() - particles[0]->getPos();
    Vector3d e1 = particles[2]->getPos() - particles[1]->getPos();
    Vector3d e2 = particles[0]->getPos() - particles[2]->getPos();
    Vector3d e3 = particles[3]->getPos() - particles[0]->getPos();
    Vector3d e4 = particles[1]->getPos() - particles[3]->getPos();
    double c01 = IsobendingConstraint::cotan(e0, -1.0*e1);
    double c02 = IsobendingConstraint::cotan(e0, -1.0*e2);
    double c03 = IsobendingConstraint::cotan(e0, e3);
    double c04 = IsobendingConstraint::cotan(e0, e4);
    VectorXd K(4);
    K << c01+c04, c02 + c03, -c01 -c02, -c03 -c04;
    double A0 = 0.5*e1.cross(e2).norm();
    double A1 = 0.5*e4.cross(e3).norm();
    Q = 3.0/(A0+A1)*K*K.transpose();

}

void IsobendingConstraint::setGradient() {
    int n_rows = particles[0]->getPos().rows();
    grad = MatrixXd::Zero(n_rows, 4);
    for (int i = 0; i < grad.cols(); i++){
        VectorXd temp = VectorXd::Zero(n_rows);
        for (int j = 0; j < grad.cols(); j++){
            //std::cout << "Q(" << i << "," << j << ") = " << Q(i,j) << std::endl;
            //std::cout << "particles[" << j << "] = " << RowVectorXd(particles[j]->getPos()) << std::endl;
            temp += Q(i,j)*particles[j]->getPos();
        }
        //std::cout << "grad.col(" << i << ") = " << RowVectorXd(temp) << std::endl;
        grad.col(i) = temp;
    }   
}

void IsobendingConstraint::update(){
    setHessian();
    double _value = 0.0;
    for (size_t i = 0; i < particles.size(); i++){
        for (size_t j = 0; j < particles.size(); j++){
            _value += Q(i,j)*particles[i]->getPos().dot(particles[j]->getPos());
        }
    }
    setValue(0.5*_value);
    //std::cout << " C_bend = " << _value << std::endl;
    setGradient();
}