 #ifndef CONSTRAINT_ISOBENDING_H
 #define CONSTRAINT_ISOBENDING_H

#include "constraint.h"

class IsobendingConstraint : public Constraint {
    protected:
        MatrixXd Q;
    public:
        IsobendingConstraint(){};
        IsobendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double alpha, double stiffness = 1.0);
        ~IsobendingConstraint(){};

        MatrixXd getHessian() const ;
        double cotan(Vector3d e1, Vector3d e2) const;
        void setHessian();
        void setGradient() override;
        void update() override;
};

#endif