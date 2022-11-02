#include <GL/glew.h>
#include <GL/freeglut.h>
#include "sphere.h"

typedef std::shared_ptr<Particle> ParticlePtr;


Sphere::Sphere(double _radius, double x, double y, double z, double _m, GLfloat* _c, VectorXd _v ) : Object(x, y, z, _m, _c, _v){
    radius = _radius;
    ParticlePtr p(new Particle(pos, _v, _m));
    vertices.push_back(p);
}

void Sphere::draw() const {
    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
    glTranslated(vertices[0]->getPos().x(), vertices[0]->getPos().y(), vertices[0]->getPos().z());
    glutSolidSphere(radius, 30, 30);
    glPopMatrix();
}