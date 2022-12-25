 #ifndef CONSTRAINT_BENDING_H
 #define CONSTRAINT_BENDING_H

#include "constraint.h"

class IsobendingConstraint : public Constraint {
    protected:
        MatrixXd Q;
    public:
        IsobendingConstraint(){};
        IsobendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double alpha, double stiffness = 1.0);
        ~IsobendingConstraint(){};

        MatrixXd getHessian() const ;
        double cotan(VectorXd e1, VectorXd e2) const;
        void setHessian();
        void setGradient() override;
        void update() override;
};

#endif