#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "particle.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

typedef std::shared_ptr<Particle> ParticlePtr;

enum constraint_type {
    DISTANCE_CONSTRAINT,
    GENERAL_CONSTRAINT
};

class Constraint {
    protected:
        constraint_type type;
        double value = 0.0;
        double stiffness = 1.0;
        double alpha = 0.0;
        double lambda = 0.0;
        std::vector<ParticlePtr> particles{};
        MatrixXd grad{};

    public:
        Constraint(){};
        Constraint(std::vector<ParticlePtr> particles, double alpha = 0.0, double stiffness = 1.0, constraint_type type = GENERAL_CONSTRAINT);
        ~Constraint(){};

        void setValue(double value);
        void setStiffness(double s);
        void setCompliance(double alpha);
        void setLagrangeMultiplier(double lambda);
        void setParticles(std::vector<ParticlePtr> particles);
        void setGradient(std::vector<ParticlePtr> particles);

        constraint_type getType() const;
        double getValue() const;
        double getStiffness() const;
        double getCompliance() const;
        double getLagrangeMultiplier() const;
        std::vector<ParticlePtr> getParticles() const;
        MatrixXd getGradient() const;

        virtual void solveConstraint(double dt);
        

};

#endif