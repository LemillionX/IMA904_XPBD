#ifndef CONSTRAINT_COLLISION_H
#define CONSTRAINT_COLLISION_H

#include "constraint.h"
#include "constraint_dist.h"

class CollisionConstraint : public DistConstraint {
    public:
        CollisionConstraint(){};
        CollisionConstraint(ParticlePtr p1, ParticlePtr p2, double l0, double alpha, double stiffness = 1.0) : DistConstraint(p1, p2, l0, alpha, stiffness )
        {type = INEQUALITY;};
        ~CollisionConstraint(){};
};


#endif
