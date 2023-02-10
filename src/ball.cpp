#include <GL/glew.h>
#include <GL/freeglut.h>
#include <imgui/imgui.h>

#include <cmath>
#include "pbd.h"
#include "camera.h"
#include "floor.h"
#include "sphere.h"
#include "cloth.h"
#include "plane.h"

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<Constraint> ConstraintPtr;
typedef std::shared_ptr<Sphere> SpherePtr;
typedef std::shared_ptr<Cloth> ClothPtr;

// Colors
GLfloat WHITE[] = {1, 1, 1};
GLfloat RED[] = {1, 0, 0};
GLfloat BLUE[] = {0, 0, 1};
GLfloat GREEN[] = {0, 1, 0};
GLfloat MAGENTA[] = {1, 0, 1};

// Simulation Settings
const int FPS = 200;
const float dt = 1000/FPS; // in ms, e.g : 1000/frame rate
const Vector3d gravity(0.0, -9.81, 0.0);
const double compliance = 0.01; // compliance coeff for XPBD
const double compliance_penetration = 0.0; // compliance coeff for XPBD
const double compliance_collision = 0.0; // compliance coeff for XPBD
const int N_ITER = 40;
const float SPEED_DAMPING = 0.999;
const int CLOTH_RESOLUTION = 15; 
const double CLOTH_SIZE = 2;
const int N_SPHERE = 1;
const double SPHERE_RADIUS = 0.2;
unsigned int TIME = 0;
std::vector<VectorXd> old_pos = {};

// Camera movement
int xOrigin = -1;
int yOrigin = -1;
double stepRot = 0.00005;

// Initialize Global Objects
Camera camera;
Floor scene_floor(20, 20);
std::vector<ObjectPtr> lst_objects;
std::vector<ConstraintPtr> constraints;
double w_max = 11;
double w_min = 9;
double h_max = 2.5;
double h_min = 0.5;
double d_max = 11;
double d_min = 7;

Vector3d cubeA(Vector3d(w_max, h_max, d_max));
Vector3d cubeB(Vector3d(w_max, h_max, d_min));
Vector3d cubeC(Vector3d(w_min, h_max, d_min));
Vector3d cubeD(Vector3d(w_min, h_max, d_max));
Vector3d cubeE(Vector3d(w_max, h_min, d_max));
Vector3d cubeF(Vector3d(w_max, h_min, d_min));
Vector3d cubeG(Vector3d(w_min, h_min, d_min));
Vector3d cubeH(Vector3d(w_min, h_min, d_max));

Plane wallLeft(cubeA, cubeB, cubeF, cubeE);
Plane wallRight(cubeD, cubeC, cubeG, cubeH);
Plane wallTop(cubeA, cubeD, cubeC, cubeB);
Plane wallBottom(cubeE, cubeH, cubeG, cubeF);
Plane wallBack(cubeA, cubeD, cubeH, cubeE);
Plane wallFront(cubeB, cubeC, cubeG, cubeF);

GLfloat* rand_color(int key){
    switch (key)
    {
    case 0:
        return WHITE;
    case 1:
        return RED;
    case 2:
        return GREEN;
    case 3:
        return MAGENTA;
    case 4:
        return BLUE;
    }
}


void init_ball(){
    SpherePtr sphere(new Sphere(SPHERE_RADIUS, 10.0 , 2.0, 9.0, 5.0, rand_color(random()%5), Vector3d(0.1*(random()%100-50),0.1*(random()%100-50),0.1*(random()%100))));

    // Gravity
    sphere->vertices[0]->setForce(sphere->vertices[0]->getMass()*gravity);

    // Sphere Floor constraint
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], Vector3d(0,0,0), Vector3d(0,0,scene_floor.getDepth()), Vector3d(scene_floor.getWidth(), 0,0), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], Vector3d(scene_floor.getWidth(), 0,scene_floor.getDepth()), Vector3d(scene_floor.getWidth(), 0,0), Vector3d(0,0,scene_floor.getDepth()), SPHERE_RADIUS, 0));

    // Bottom wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallBottom.getP0(), wallBottom.getP3(), wallBottom.getP1(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallBottom.getP1(), wallBottom.getP3(), wallBottom.getP2(), SPHERE_RADIUS, 0));
    // Top wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallTop.getP0(), wallTop.getP1(), wallTop.getP3(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallTop.getP1(), wallTop.getP2(), wallTop.getP3(), SPHERE_RADIUS, 0));
    // Right wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallRight.getP0(), wallRight.getP3(), wallRight.getP1(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallRight.getP2(), wallRight.getP1(), wallRight.getP3(), SPHERE_RADIUS, 0));
    // Left wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallLeft.getP0(), wallLeft.getP1(), wallLeft.getP3(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallLeft.getP2(), wallLeft.getP3(), wallLeft.getP1(), SPHERE_RADIUS, 0));
    // Back wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallBack.getP0(), wallBack.getP3(), wallBack.getP1(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallBack.getP2(), wallBack.getP1(), wallBack.getP3(), SPHERE_RADIUS, 0));
    // Front wall collision
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallFront.getP0(), wallFront.getP1(), wallFront.getP3(), SPHERE_RADIUS, 0));
    constraints.push_back(std::make_shared<WallConstraint>(sphere->vertices[0], wallFront.getP2(), wallFront.getP3(), wallFront.getP1(), SPHERE_RADIUS, 0));

    for (int i =0; i < lst_objects.size(); i++){
        // Sphere collision
        constraints.push_back(std::make_shared<CollisionConstraint>(lst_objects[i]->vertices[0], sphere->vertices[0], 2.1*SPHERE_RADIUS, compliance_collision));
    }


    lst_objects.push_back(sphere);
}

void init_objects()
{
    for (int i =0; i < N_SPHERE; i++){
        init_ball();
    }
}


void init_constraints()
{


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

    camera.setX(scene_floor.centerx());
    camera.setZ(scene_floor.centerz()-6);

    init_objects();
    init_constraints();
}


void update(double _dt){
    // std::cout << "dt = " << _dt << std::endl;
    // std::cout << "step = " << TIME++ << std::endl;
    old_pos.clear();

    // Predicted position
    for (auto &obj : lst_objects){
        for (auto& vertex: obj->vertices){
            old_pos.push_back(vertex->getPos()); // Store old position
            vertex->setSpeed(vertex->getSpeed() + _dt*vertex->getForce()*vertex->getInvMass()); // Implicit Euler
            vertex->setPos(vertex->getPos() + _dt*vertex->getSpeed()); // Implicit Euler
        }
    }

    // std::cout << " Ball position = " << lst_objects[0]->vertices[0]->getPos().transpose() << std::endl;
    // Initialize Lagrange Multiplier
    for (auto& c : constraints){
        c->setLagrangeMultiplier(0);
    }

    // Solving constraints
    for (int i = 0; i < N_ITER; i++)
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
            vertex->setSpeed(SPEED_DAMPING*vertex->getSpeed());// Damping for speed
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
              camera.getTargetX(), camera.getTargetY(),  camera.getTargetZ(),
              camera.getUpX(), camera.getUpY(), camera.getUpZ());

    scene_floor.draw();
    wallLeft.draw();
    wallRight.draw();
    wallTop.draw();
    wallBottom.draw();

    for (auto &obj : lst_objects)
    {
        obj->draw();
    }

    glFlush();
    glutSwapBuffers();
    update(dt/1000); // dt/1000 because dt is in ms

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
    glutTimerFunc(TIME, timer, v);
    glutPostRedisplay();

}

// Update the camera rotation when the mouse is moving
void mouseMove(int x, int y){
    if (xOrigin >= 0){
        double deltaAngleX = (x-xOrigin)*stepRot;
        //std::cout << " x = " << x << ", < xOrigin = " << xOrigin << ", delta =  " << deltaAngleX << std::endl;
        camera.setAngleX(camera.getAngleX() + deltaAngleX);
    }
    if (yOrigin >= 0){
        double deltaAngleY = (y-yOrigin)*stepRot;
        //std::cout << " y = " << y << ", < yOrigin = " << yOrigin << ", delta =  " << deltaAngleY << std::endl;
        camera.setAngleY(camera.getAngleY() + deltaAngleY);
    }
}

// Called each time a button of the mouse is pressed
void mouseButton(int key, int state, int x, int y){
    if (key == GLUT_LEFT_BUTTON){
        if (state == GLUT_UP){
            xOrigin = -1;
            yOrigin = -1;
        } 
        else {
            xOrigin = x;
            yOrigin = y;
        }
    }
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
        if (glutGetModifiers() == GLUT_ACTIVE_CTRL){
            camera.moveForward();
        } else {
            camera.moveUp();
        }
        break;
    case GLUT_KEY_DOWN:
        if (glutGetModifiers() == GLUT_ACTIVE_CTRL){
            camera.moveBack();
        } else {
            camera.moveDown();
        }
        break;
    case GLUT_KEY_F1:
        init_ball();
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

void test_isobending(){ 
    std::cout << "Testing isobending constraint..." << std::endl;

    std::cout << "Initialising position...";
    VectorXd pos1(3), pos2(3), pos3(3), pos4(3);
    pos1 << 0.0, 0.0, 0.0;
    pos2 << 0.0, 1.0, 0.0;
    pos3 << -0.5, 0.5, 0.0;
    pos4 << 0.5, 0.5, 0.0;
    std::cout << "OK" << std::endl;

    std::cout << "Initialising particles...";
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(3),2.0));
    ParticlePtr p2(new Particle(pos2, VectorXd::Zero(3),1.0));
    ParticlePtr p3(new Particle(pos3, VectorXd::Zero(3),1.0));
    ParticlePtr p4(new Particle(pos4, VectorXd::Zero(3),1.0));
    std::cout << "OK" << std::endl;

    std::cout << "Initialising constraint...";
    IsobendingConstraint c_bend(p1, p2, p3, p4, 0.0);
    std::cout << "OK" << std::endl;

    int n_iter = 1;
    std::cout << "Solving constraint...";
    for (int i = 0; i < n_iter; i++){
        c_bend.solveConstraint(0.01);
    }
    std::cout << "OK" << std::endl;

    std::cout << "************** Final state **************** " << std::endl;
    for (size_t i = 0; i < c_bend.getParticles().size(); i++){
        std::cout << "Particle " << i << " position : " << RowVectorXd(c_bend.getParticles()[i]->getPos()) << std::endl;
    }
}

void test_bending(){
    std::cout << "Testing bending constraint..." << std::endl;

    std::cout << "Initialising position...";
    VectorXd pos1(3), pos2(3), pos3(3), pos4(3);
    pos1 << 0.0, 0.0, 0.0;
    pos2 << 0.0, 1.0, 0.0;
    pos3 << -0.5, 0.5, 0.0;
    pos4 << 0.5, 0.5, 0.0;
    std::cout << "OK" << std::endl;

    std::cout << "Initialising particles...";
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(3),2.0));
    ParticlePtr p2(new Particle(pos2, VectorXd::Zero(3),1.0));
    ParticlePtr p3(new Particle(pos3, VectorXd::Zero(3),1.0));
    ParticlePtr p4(new Particle(pos4, VectorXd::Zero(3),1.0));
    std::cout << "OK" << std::endl;

    std::cout << "Initialising constraint...";
    BendingConstraint c_bend(p1, p2, p3, p4, 0.0);
    std::cout << "OK" << std::endl;

    int n_iter = 1;
    std::cout << "Solving constraint...";
    for (int i = 0; i < n_iter; i++){
        c_bend.solveConstraint(0.01);
    }
    std::cout << "OK" << std::endl;

    std::cout << "************** Final state **************** " << std::endl;
    for (size_t i = 0; i < c_bend.getParticles().size(); i++){
        std::cout << "Particle " << i << " position : " << RowVectorXd(c_bend.getParticles()[i]->getPos()) << std::endl;
    }
}


int main(int argc, char **argv)
{
    std::cout << " Here is a ball scene !" << std::endl;
    // Init GLUT and create window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(800, 600);
    glutCreateWindow("XPBD");

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutSpecialFunc(special);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMove);

    glutTimerFunc(100, timer, 0);

    // Initialise the scene
    init();

    // Enter the main loop
    glutMainLoop();

    return 0;
}