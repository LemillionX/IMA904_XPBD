#include "pbd.h"

MatrixXd PBD::initPosMatrix(std::vector<particle> vertices){
    int n_rows = vertices.size();
    int n_cols = vertices[0].p.size();
    MatrixXd x(n_rows,n_cols);

    for (int i = 0; i < n_rows; i++){
        x.row(i) = vertices[i].p;
    }

    return x;
}