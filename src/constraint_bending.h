 #ifndef CONSTRAINT_BENDING_H
 #define CONSTRAINT_BENDING_H

#include "constraint.h"

class BendingConstraint : public Constraint {
    protected:
        double phi0=0;
    public:
        BendingConstraint(){};
        BendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double alpha, double stiffness = 1.0);
        ~BendingConstraint(){};

        double getDihedralAngle() const ;
        MatrixXd crossProdMat(VectorXd _p) const;
        MatrixXd crossProdGrad_p1(Vector3d _p1, Vector3d _p2) const;
        MatrixXd crossProdGrad_p2(Vector3d _p1, Vector3d _p2) const;
        void setGradient() override;
        void update() override;
};

#endif