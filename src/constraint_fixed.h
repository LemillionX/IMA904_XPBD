#ifndef CONSTRAINT_FIXED_H
#define CONSTRAINT_FIXED_H

#include "constraint.h"

class FixedConstraint : public Constraint {
    protected:
        VectorXd rest_pos{};
    public:
        FixedConstraint(){};
        FixedConstraint(ParticlePtr p, VectorXd p0, double alpha = 0.00001, double stiffness = 1.0);
        ~FixedConstraint(){};

        void setGradient() override;
        void update() override;
};

#endif