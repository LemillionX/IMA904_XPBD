#include "cloth.h"
#include <iostream>

typedef std::shared_ptr<Particle> ParticlePtr;

Cloth::Cloth(int _width, int _length, double x, double y, double z, double _m, GLfloat *_c, double _step, VectorXd _v) : Object(x, y, z, _m, _c, _v)
{
    width = _width;
    length = _length;
    step = _step;
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < width; j++)
        {
            vertices.push_back(std::make_shared<Particle>(Particle(pos + Vector3d(j*step, -i*step, 0), _v, _m/(width*length))));
        }
    }
}

void Cloth::draw() const
{
    GLfloat WHITE[] = {1, 1, 1};
    GLfloat BLUE[] = {0, 0, 1};
    glBegin(GL_QUADS);
    glNormal3d(0, 1, 0);
    int i; // count
    for (int x = 1; x < length; x++)
    {
        i = x * width;
        for (int z = 0; z < width - 1; z++)
        {
            //glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, (x + z) % 2 == 0 ? BLUE : WHITE);
            //std::cout << "i = " << i << "/" << vertices.size() << std::endl;
            glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
            glVertex3d(vertices[i]->getPos().x(), vertices[i]->getPos().y(), vertices[i]->getPos().z());
            glVertex3d(vertices[i + 1]->getPos().x(), vertices[i + 1]->getPos().y(), vertices[i + 1]->getPos().z());
            glVertex3d(vertices[i - width + 1]->getPos().x(), vertices[i - width + 1]->getPos().y(), vertices[i - width + 1]->getPos().z());
            glVertex3d(vertices[i - width]->getPos().x(), vertices[i - width]->getPos().y(), vertices[i - width]->getPos().z());
            i++;
        }
    }
    glEnd();
}
