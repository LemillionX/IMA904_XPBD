#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "particle.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

typedef std::shared_ptr<Particle> ParticlePtr;

enum constraint_type {
    EQUALITY,
    INEQUALITY,
    FIXED_CONSTRAINT
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
        Constraint(double alpha, double stiffness = 1.0, constraint_type type = EQUALITY);
        Constraint(std::vector<ParticlePtr> particles, double alpha = 0.0, double stiffness = 1.0, constraint_type type = EQUALITY);
        virtual ~Constraint(){};

        void setValue(double value);
        void setStiffness(double s);
        void setCompliance(double alpha);
        void setLagrangeMultiplier(double lambda);
        void setParticles(std::vector<ParticlePtr> particles);
        virtual void setGradient();

        constraint_type getType() const;
        double getValue() const;
        double getStiffness() const;
        double getCompliance() const;
        double getLagrangeMultiplier() const;
        std::vector<ParticlePtr> getParticles() const;
        MatrixXd getGradient() const;

        virtual void update() = 0;
        virtual void solveConstraint(double dt);
        

};

#endif