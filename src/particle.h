#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>
using namespace Eigen;

struct particle {
    VectorXd p; // position
    VectorXd v; // speed
    double m; // mass
    double w; // inverse mass
};




#endif