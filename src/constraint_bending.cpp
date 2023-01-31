#include "constraint_bending.h"
#include <iostream>
#include <vector>

BendingConstraint::BendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double alpha, double stiffness) : Constraint(alpha, stiffness, EQUALITY){
    std::vector<ParticlePtr> _particles = {p1, p2, p3, p4};
    setParticles(_particles);
    Vector3d _p2 = particles[1]->getPos()-particles[0]->getPos();
    Vector3d _p3 = particles[2]->getPos() - particles[0]->getPos();
    Vector3d _p4 = particles[3]->getPos()-particles[0]->getPos();

    VectorXd n1 = _p2.cross(_p3).normalized();
    VectorXd n2 = _p2.cross(_p4).normalized();
    phi0 = acos(n1.dot(n2));
    update();
}


double BendingConstraint::getDihedralAngle() const{
    return phi0;
}

MatrixXd BendingConstraint::crossProdMat(VectorXd _p) const{
    MatrixXd crossMat(3,3);
    crossMat << 0, -_p.z(), _p.y(),
                _p.z(), 0, -_p.x(),
                -_p.y(), _p.x(), 0;
    return crossMat;
}

MatrixXd BendingConstraint::crossProdGrad_p1(Vector3d _p1, Vector3d _p2) const{
    Vector3d _n = _p1.cross(_p2);
    double crossNorm = _n.norm();
    _n = _n.normalized();
    MatrixXd _pTilde = crossProdMat(_p2);
    return (-_pTilde + _n*(_n.cross(_p2)).transpose())/crossNorm;
    
}

MatrixXd BendingConstraint::crossProdGrad_p2(Vector3d _p1, Vector3d _p2) const{
    Vector3d _n = _p1.cross(_p2);
    double crossNorm = _n.norm();
    _n = _n.normalized();
    MatrixXd _pTilde = crossProdMat(_p1);
    return -(-_pTilde + _n*(_n.cross(_p1)).transpose())/crossNorm;
    
}


void BendingConstraint::setGradient() {
    grad = MatrixXd::Zero(3, 4);

    Vector3d p2 = particles[1]->getPos()-particles[0]->getPos();
    Vector3d p3 = particles[2]->getPos() - particles[0]->getPos();
    Vector3d p4 = particles[3]->getPos()-particles[0]->getPos();

    // Compute n1 = p2 x p3, n2 = p2 x p4 and d = n1.n2
    VectorXd n1 = p2.cross(p3);
    n1 = n1.normalized();
    VectorXd n2 = p2.cross(p4);
    n2 = n2.normalized();
    double d = n1.dot(n2);

    // Compute gradients
    if (d*d < 1.0){

        // Compute partial derivatives
        MatrixXd grad_N1P2 = crossProdGrad_p1(p2, p3);
        MatrixXd grad_N1P3 = crossProdGrad_p2(p2, p3);
        MatrixXd grad_N2P2 = crossProdGrad_p1(p2, p4);
        MatrixXd grad_N2P4 = crossProdGrad_p2(p2, p4);

        double coeff = -1.0/sqrt(1.0-d*d);
        grad.col(2) = coeff * grad_N1P3.transpose()*n2;
        grad.col(3) = coeff * grad_N2P4.transpose()*n1;
        grad.col(1) = coeff * (grad_N1P2.transpose()*n2 + grad_N2P2.transpose()*n1);
        grad.col(0) = -grad.col(1) - grad.col(2) - grad.col(3);
    }

}

void BendingConstraint::update(){
    Vector3d _p2 = particles[1]->getPos()-particles[0]->getPos();
    Vector3d _p3 = particles[2]->getPos()-particles[0]->getPos();
    Vector3d _p4 = particles[3]->getPos()-particles[0]->getPos();

    VectorXd n1 = _p2.cross(_p3);
    n1 = n1.normalized();
    VectorXd n2 = _p2.cross(_p4);
    n2 = n2.normalized();

    double _value = acos(n1.dot(n2)) - phi0;
    setValue(_value);
    setGradient();
}