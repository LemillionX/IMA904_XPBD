#include "constraint_penetration.h"
#include <iostream>
#include <vector>

PenetrationConstraint::PenetrationConstraint(ParticlePtr q, ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, double _h, double alpha, double stiffness) : Constraint(alpha, stiffness, INEQUALITY){
    std::vector<ParticlePtr> _particles = {q, p1, p2, p3};
    setParticles(_particles);
    h=_h;
    update();
}

double PenetrationConstraint::getThickness() const {
    return h;
}

MatrixXd PenetrationConstraint::crossProdMat(VectorXd _p) const{
    MatrixXd crossMat(3,3);
    crossMat << 0, -_p.z(), _p.y(),
                _p.z(), 0, -_p.x(),
                -_p.y(), _p.x(), 0;
    return crossMat;
}

MatrixXd PenetrationConstraint::crossProdGrad_p1(Vector3d _p1, Vector3d _p2) const {
    MatrixXd _pTilde = crossProdMat(_p2);
    Vector3d _n = _p1.cross(_p2);
    double crossNorm = _n.norm();
    _n = _n.normalized();
    return (- _pTilde + _n*(_n.cross(_p2).transpose()))/crossNorm;
}

MatrixXd PenetrationConstraint::crossProdGrad_p2(Vector3d _p1, Vector3d _p2) const {
    MatrixXd _pTilde = crossProdMat(_p1);
    Vector3d _n = _p1.cross(_p2);
    double crossNorm = _n.norm();
    _n = _n.normalized();
    return -(- _pTilde + _n*(_n.cross(_p1).transpose()))/crossNorm;
}

void PenetrationConstraint::setGradient(){
    grad = MatrixXd::Zero(3,4);
    
    Vector3d _q = particles[0]->getPos() - particles[1]->getPos();
    Vector3d _p21 = particles[2]->getPos() - particles[1]->getPos();
    Vector3d _p31 = particles[3]->getPos() - particles[1]->getPos();

    Vector3d _n = _p21.cross(_p31).normalized();
    MatrixXd gradP2 = crossProdGrad_p1(_p21, _p31);
    MatrixXd gradP3 = crossProdGrad_p2(_p21, _p31);

    grad.col(0) = _n;
    grad.col(2) = gradP2.transpose()*_q;
    grad.col(3) = gradP3.transpose()*_q;
    grad.col(1) = -grad.col(0) - grad.col(2) - grad.col(3);
}

void PenetrationConstraint::update(){
    Vector3d _q = particles[0]->getPos() - particles[1]->getPos();
    Vector3d _p21 = particles[2]->getPos() - particles[1]->getPos();
    Vector3d _p31 = particles[3]->getPos() - particles[1]->getPos();
    Vector3d _n = _p21.cross(_p31).normalized();
    
    setValue(_q.dot(_n) - h);
    setGradient();
}