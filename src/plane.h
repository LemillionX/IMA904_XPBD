#ifndef PLANE_H
#define PLANE_H

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <Eigen/Dense>
using namespace Eigen;

class Plane {
  Vector3d p0, p1, p2, p3;

public:
  Plane(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3): p0(p0), p1(p1), p2(p2), p3(p3) {}

  void draw() {
    // Colors
    GLfloat BLUE[] = {0, 0, 1};

    glBegin(GL_QUADS);
    glNormal3d(0, 1, 0);

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,BLUE);
    glVertex3d(p3.x(), p3.y(), p3.z());
    glVertex3d(p2.x(), p2.y(), p2.z());
    glVertex3d(p1.x(), p1.y(), p1.z());
    glVertex3d(p0.x(), p0.y(), p0.z());


    glEnd();
    glEndList();
  }

  Vector3d getP0() const {return p0;}
  Vector3d getP1() const {return p1;}
  Vector3d getP2() const {return p2;}
  Vector3d getP3() const {return p3;}
};


#endif