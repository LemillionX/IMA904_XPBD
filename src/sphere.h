#ifndef SPHERE_H
#define SPHERE_H

#include "object.h"

class Sphere : public Object {
    double radius;

    public:
        Sphere(){};
        Sphere(double radius, double x, double y, double z, double m, GLfloat* c, VectorXd _v = VectorXd::Zero(3));
        ~Sphere(){};

        void draw() const override ;
        double getRadius() const {return radius;}
};

#endif