 #ifndef CONSTRAINT_DIST_H
 #define CONSTRAINT_DIST_H

#include "constraint.h"

class DistConstraint : public Constraint {
    protected:
        ParticlePtr particle1;
        ParticlePtr particle2;
    public:
        DistConstraint(){};
        DistConstraint(ParticlePtr p1, ParticlePtr p2, double alpha, double stiffness = 1.0);
        ~DistConstraint(){};
};

#endif