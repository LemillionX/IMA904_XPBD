#ifndef CLOTH_H
#define CLOTH_H

#include <GL/glew.h>
#include <GL/freeglut.h>
#include "object.h"

class Cloth : public Object {
    int displayListId;
    int width;
    int length;
    double step;
    public:
        Cloth(int _width, int _length, double x, double y, double z, double _m, GLfloat* c, double step = 0.1, VectorXd v = VectorXd::Zero(3));
        ~Cloth(){};

        int getWidth(){return width;};
        int getLength(){return length;};
        double getStep() {return step;};
        void draw() const override;
};


#endif