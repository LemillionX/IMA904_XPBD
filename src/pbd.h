#ifndef PBD_H
#define PBD_H

#include <iostream> 
#include <vector>
#include "particle.h"

class PBD {
    public:
        MatrixXd initPosMatrix(std::vector<particle> vertices);
        MatrixXd initDistConstaint(std::vector<particle> vertices, const double l0);
        void solveConstraints(MatrixXd points, MatrixXd constraints, const double stiffness = 1.0);

};


#endif