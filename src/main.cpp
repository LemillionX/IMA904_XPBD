#include "pbd.h"

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

int main(){
    std::cout << " Hello world !" << std::endl;

    return 0;
}