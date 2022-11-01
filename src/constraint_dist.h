 #ifndef CONSTRAINT_DIST_H
 #define CONSTRAINT_DIST_H

#include "constraint.h"

class DistConstraint : public Constraint {
    protected:
        double l0;
    public:
        DistConstraint(){};
        DistConstraint(ParticlePtr p1, ParticlePtr p2, double l0, double alpha, double stiffness = 1.0);
        ~DistConstraint(){};

        double getRestLength() const ;
        void setGradient(std::vector<ParticlePtr> particles) override;
        void update() override;
};

#endif