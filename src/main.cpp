#include "pbd.h"

int main(){
    std::cout << " Hello world !" << std::endl;
    VectorXd v(3);
    v << 1,2,3;
    std::cout << v << std::endl;
    return 0;
}