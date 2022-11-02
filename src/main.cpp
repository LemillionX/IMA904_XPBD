#include <GL/glew.h>
#include <GL/freeglut.h>

#include <cmath>
#include "pbd.h"
#include "camera.h"
#include "floor.h"
#include "sphere.h"

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<Sphere> SpherePtr;

// Colors
GLfloat WHITE[] = {1, 1, 1};
GLfloat RED[] = {1, 0, 0};
GLfloat BLUE[] = {0, 0, 1};
GLfloat GREEN[] = {0, 1, 0};
GLfloat MAGENTA[] = {1, 0, 1};

// Initialize Objects
const float dt = 1000 / 60; // in ms, e.g : 1000/frame rate
const Vector3d gravity(0.0, -9.81, 0.0);
const double compliance = 0.01; // compliance coeff for XPBD

Camera camera;
Floor scene_floor(20, 20);
std::vector<ObjectPtr> lst_objects;
std::vector<ConstraintPtr> constraints;

void init_objects()
{
    SpherePtr sphere1(new Sphere(0.2, 10.0, 1.0, 10.0, 1.0, GREEN));
    SpherePtr sphere2(new Sphere(0.2, 10.0, 3.0, 10.0, 1.0, BLUE));
    sphere1->vertices[0]->setForce(gravity);
    sphere2->vertices[0]->setForce(gravity);

    lst_objects.push_back(sphere1);
    lst_objects.push_back(sphere2);
}

void init_constraints()
{
    constraints.push_back(std::make_shared<DistConstraint>(lst_objects[0]->vertices[0], lst_objects[1]->vertices[0], 1.0, compliance));
    constraints.push_back(std::make_shared<FixedConstraint>(lst_objects[0]->vertices[0], lst_objects[0]->vertices[0]->getPos()));
}

// Application-specific initialization: Set up global lighting parameters and create display lists.
void init()
{
    glEnable(GL_DEPTH_TEST);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, WHITE);
    glLightfv(GL_LIGHT0, GL_SPECULAR, WHITE);
    glMaterialfv(GL_FRONT, GL_SPECULAR, WHITE);
    glMaterialf(GL_FRONT, GL_SHININESS, 30);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    scene_floor.create();
    init_objects();
    init_constraints();
}


void update(double _dt){
    std::vector<VectorXd> old_pos = {};

    // Predicted position
    for (auto &obj : lst_objects){
        for (auto& vertex: obj->vertices){
            old_pos.push_back(vertex->getPos()); // Store old position
            vertex->setSpeed(vertex->getSpeed() + _dt*vertex->getForce()); // Implicit Euler
            vertex->setPos(vertex->getPos() + _dt*vertex->getSpeed()); // Implicit Euler
        }
    }

    // Initialize Lagrange Multiplier
    for (auto& c : constraints){
        c->setLagrangeMultiplier(0);
    }

    // Solving constraints
    int n_iter = 20;
    for (int i = 0; i < n_iter; i++)
    {
        for (auto& c : constraints){
            c->solveConstraint(_dt);
        }
    }
    // Updating positions & speed
    // Positions are already updated when solving the constraints
    int count = 0;
    for (auto &obj : lst_objects){
        for (auto& vertex: obj->vertices){
            vertex->setSpeed((vertex->getPos() - old_pos[count])/_dt);
            count++;
        }
    }


}


// Draws one frame, the floor from the current camera position.
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(camera.getX(), camera.getY(), camera.getZ(),
              scene_floor.centerx(), 0.0, scene_floor.centerz(),
              0.0, 1.0, 0.0);
    scene_floor.draw();


    for (auto &obj : lst_objects)
    {
        obj->draw();
    }

    update(dt/1000); // dt/1000 because dt is in ms


    glFlush();
    glutSwapBuffers();
}

// On reshape, constructs a camera that perfectly fits the window.
void reshape(GLint w, GLint h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, GLfloat(w) / GLfloat(h), 1.0, 150.0);
    glMatrixMode(GL_MODELVIEW);
}

// Requests to draw the next frame.
void timer(int v)
{
    glutPostRedisplay();
    glutTimerFunc(dt, timer, v);
}

// Moves the camera according to the key pressed, then ask to refresh the display.
void special(int key, int, int)
{
    switch (key)
    {
    case GLUT_KEY_LEFT:
        camera.moveLeft();
        break;
    case GLUT_KEY_RIGHT:
        camera.moveRight();
        break;
    case GLUT_KEY_UP:
        camera.moveUp();
        break;
    case GLUT_KEY_DOWN:
        camera.moveDown();
        break;
    }
    glutPostRedisplay();
}

void test_build_eigen()
{
    std::cout << " Test build eigen : " << std::endl;
    VectorXd v(3);
    v << 1, 2, 3;
    std::cout << v << std::endl;
}

void test_particle_class()
{
    VectorXd pos1(2);
    pos1 << 0.0, 0.0;
    VectorXd pos2(2);
    pos2 << 0.0, -1.0;
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(2), 2.0));
    std::cout << " Test Particle class : " << std::endl;
    std::cout << " pos : " << RowVectorXd(p1->getPos()) << std::endl;
    std::cout << " speed : " << RowVectorXd(p1->getSpeed()) << std::endl;
    std::cout << " mass : " << p1->getMass() << std::endl;
    std::cout << " inverse_mass : " << p1->getInvMass() << std::endl;
    std::cout << " external forces : " << RowVectorXd(p1->getForce()) << std::endl;
}

void test_init_dist_constraint()
{
    std::cout << " Test: Initialising Distance Constraint  " << std::endl;
    VectorXd pos1(2);
    pos1 << 0.0, 0.0;
    VectorXd pos2(2);
    pos2 << 0.0, -2.0;
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(2), 2.0));
    ParticlePtr p2(new Particle(pos2, VectorXd::Zero(2), 2.0));
    DistConstraint c1(p1, p2, 1.0, 1.0);

    std::cout << " Particle 1 position : " << RowVectorXd(c1.getParticles()[0]->getPos()) << std::endl;
    std::cout << " Particle 2 position : " << RowVectorXd(c1.getParticles()[1]->getPos()) << std::endl;
    std::cout << " Rest length : " << c1.getRestLength() << std::endl;
    std::cout << " Constraint value : " << c1.getValue() << std::endl;
    std::cout << " Gradient : \n " << c1.getGradient() << std::endl;
    std::cout << " Constraint stiffness : " << c1.getStiffness() << std::endl;
    std::cout << " Constraint compliance : " << c1.getCompliance() << std::endl;
    std::cout << " Constraint multiplier : " << c1.getLagrangeMultiplier() << std::endl;
}

void test_solve_dist_constraint()
{
    std::cout << " Test: Solving Distance Constraint  " << std::endl;
    VectorXd pos1(1);
    pos1 << 0.0;
    VectorXd pos2(1);
    pos2 << -2.0;
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(1), 2.0));
    ParticlePtr p2(new Particle(pos2, VectorXd::Zero(1), 2.0));
    DistConstraint c1(p1, p2, 1.0, 0.00);
    int n_iter = 20;

    std::cout << "************** Initial state **************** " << std::endl;
    std::cout << " Particle 1 position : " << RowVectorXd(c1.getParticles()[0]->getPos()) << std::endl;
    std::cout << " Particle 2 position : " << RowVectorXd(c1.getParticles()[1]->getPos()) << std::endl;
    std::cout << " Gradient : \n " << c1.getGradient() << std::endl;

    std::cout << "Solving...";
    for (int i = 0; i < n_iter; i++)
    {
        c1.solveConstraint(0.01);
    }

    std::cout << "done " << std::endl;
    std::cout << "************** Final state **************** " << std::endl;
    std::cout << " Particle 1 position : " << RowVectorXd(c1.getParticles()[0]->getPos()) << std::endl;
    std::cout << " Particle 2 position : " << RowVectorXd(c1.getParticles()[1]->getPos()) << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << " Hello world !" << std::endl;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("XPBD");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutSpecialFunc(special);
    glutTimerFunc(100, timer, 0);
    init();
    glutMainLoop();

    return 0;
}