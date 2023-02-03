#ifndef CONSTRAINT_WALL_H
#define CONSTRAINT_WALL_H

#include "constraint.h"

class WallConstraint : public Constraint {
    protected:
        Vector3d p1 = Vector3d(0,0,0);
        Vector3d p2 = Vector3d(0,0,0);
        Vector3d p3= Vector3d(0,0,0);
        double h = 0.01;
    public:
        WallConstraint(){};
        WallConstraint(ParticlePtr q, Vector3d p1, Vector3d p2, Vector3d p3, double h, double alpha, double stiffness = 1.0);
        ~WallConstraint(){};

        double getThickness() const;
        void setGradient() override;
        void update() override;
};


#endif