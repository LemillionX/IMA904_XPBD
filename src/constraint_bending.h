 #ifndef CONSTRAINT_BENDING_H
 #define CONSTRAINT_BENDING_H

#include "constraint.h"

class BendingConstraint : public Constraint {
    protected:
        double phi0;
    public:
        BendingConstraint(){};
        BendingConstraint(ParticlePtr p1, ParticlePtr p2, ParticlePtr p3, ParticlePtr p4, double phi0, double alpha, double stiffness = 1.0);
        ~BendingConstraint(){};

        double getDihedralAngle() const ;
        void setGradient() override;
        void update() override;
};

#endif