#include <GL/glew.h>
#include <GL/freeglut.h>

#include <cmath>
#include "pbd.h"
#include "camera.h"
#include "floor.h"

// Colors
GLfloat WHITE[] = {1, 1, 1};
GLfloat RED[] = {1, 0, 0};
GLfloat GREEN[] = {0, 1, 0};
GLfloat MAGENTA[] = {1, 0, 1};

// Initialize Objects
Camera camera;
Floor scene_floor(20, 20);

// Application-specific initialization: Set up global lighting parameters and create display lists.
void init() {
  glEnable(GL_DEPTH_TEST);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, WHITE);
  glLightfv(GL_LIGHT0, GL_SPECULAR, WHITE);
  glMaterialfv(GL_FRONT, GL_SPECULAR, WHITE);
  glMaterialf(GL_FRONT, GL_SHININESS, 30);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  scene_floor.create();

}

// Draws one frame, the floor from the current camera position.
void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(camera.getX(), camera.getY(), camera.getZ(),
            scene_floor.centerx(), 0.0, scene_floor.centerz(),
            0.0, 1.0, 0.0);
  scene_floor.draw();
  glFlush();
  glutSwapBuffers();
}

// On reshape, constructs a camera that perfectly fits the window.
void reshape(GLint w, GLint h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(40.0, GLfloat(w) / GLfloat(h), 1.0, 150.0);
  glMatrixMode(GL_MODELVIEW);
}

// Requests to draw the next frame.
void timer(int v) {
  glutPostRedisplay();
  glutTimerFunc(1000/60, timer, v);
}

// Moves the camera according to the key pressed, then ask to refresh the display.
void special(int key, int, int) {
  switch (key) {
    case GLUT_KEY_LEFT: camera.moveLeft(); break;
    case GLUT_KEY_RIGHT: camera.moveRight(); break;
    case GLUT_KEY_UP: camera.moveUp(); break;
    case GLUT_KEY_DOWN: camera.moveDown(); break;
  }
  glutPostRedisplay();
}

void test_build_eigen(){
    std::cout << " Test build eigen : " << std::endl;
    VectorXd v(3);
    v << 1,2,3;
    std::cout << v << std::endl;
}

void test_particle_class(){
    VectorXd pos1(2); pos1 << 0.0, 0.0 ;
    VectorXd pos2(2); pos2 << 0.0, -1.0;
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(2), 2.0));
    std::cout << " Test Particle class : " << std::endl;
    std::cout << " pos : " << RowVectorXd(p1->getPos()) << std::endl;
    std::cout << " speed : " << RowVectorXd(p1->getSpeed()) << std::endl;
    std::cout << " mass : " << p1->getMass() << std::endl;
    std::cout << " inverse_mass : " << p1->getInvMass() << std::endl;
    std::cout << " external forces : " << RowVectorXd(p1->getForce()) << std::endl;

}

void test_init_dist_constraint(){
    std::cout << " Test: Initialising Distance Constraint  " << std::endl;
    VectorXd pos1(2); pos1 << 0.0, 0.0 ;
    VectorXd pos2(2); pos2 << 0.0, -2.0;
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

void test_solve_dist_constraint(){
    std::cout << " Test: Solving Distance Constraint  " << std::endl;
    VectorXd pos1(1); pos1 << 0.0 ;
    VectorXd pos2(1); pos2 << -2.0;
    ParticlePtr p1(new Particle(pos1, VectorXd::Zero(1), 2.0));
    ParticlePtr p2(new Particle(pos2, VectorXd::Zero(1), 2.0));
    DistConstraint c1(p1, p2, 1.0, 0.00);
    int n_iter = 20;

    std::cout << "************** Initial state **************** " << std::endl;
    std::cout << " Particle 1 position : " << RowVectorXd(c1.getParticles()[0]->getPos()) << std::endl;
    std::cout << " Particle 2 position : " << RowVectorXd(c1.getParticles()[1]->getPos()) << std::endl;
    std::cout << " Gradient : \n " << c1.getGradient() << std::endl;

    std::cout << "Solving...";
    for(int i = 0; i < n_iter; i++){
        c1.solveConstraint(0.01);
    }

    std::cout << "done " << std::endl;
    std::cout << "************** Final state **************** " << std::endl;
    std::cout << " Particle 1 position : " << RowVectorXd(c1.getParticles()[0]->getPos()) << std::endl;
    std::cout << " Particle 2 position : " << RowVectorXd(c1.getParticles()[1]->getPos()) << std::endl;
}

int main(int argc, char** argv){
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