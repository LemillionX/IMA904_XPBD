#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>
using namespace Eigen;

class Particle {
    private:
        VectorXd p{}; // position
        VectorXd v{}; // speed
        double m{}; // mass
        double w{}; // inverse mass
        VectorXd forces{}; // external forces

    public:
        Particle(){};
        Particle(VectorXd p, VectorXd v, double m);
        ~Particle(){};

        void setPos(VectorXd p);
        void setSpeed(VectorXd v);
        void setMass(double m);
        void setForce(VectorXd forces);
        
        VectorXd getPos() const;
        VectorXd getSpeed() const;
        double getMass() const;
        double getInvMass() const;
        VectorXd getForce() const;

};




#endif