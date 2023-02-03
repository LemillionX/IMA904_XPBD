#ifndef CONSTRAINT_PENETRATION_H
#define CONSTRAINT_PENETRATION_H

#include "constraint.h"
class PenetrationConstraint : public Constraint {
    protected:
        double h = 0.0001;
    public:
        PenetrationConstraint(){};
        PenetrationConstraint(ParticlePtr q, ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, double h, double alpha, double stiffness = 1.0);
        ~PenetrationConstraint(){};

        double getThickness() const ;
        MatrixXd crossProdMat(VectorXd _p) const;
        MatrixXd crossProdGrad_p1(Vector3d _p1, Vector3d _p2) const;
        MatrixXd crossProdGrad_p2(Vector3d _p1, Vector3d _p2) const;
        void setGradient() override;
        void update() override;        
};

#endif