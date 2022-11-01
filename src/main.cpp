#include "pbd.h"

void test_build_eigen(){
    std::cout << " Test build eigen : " << std::endl;
    VectorXd v(3);
    v << 1,2,3;
    std::cout << v << std::endl;
}

int main(){
    std::cout << " Hello world !" << std::endl;
    
    return 0;
}