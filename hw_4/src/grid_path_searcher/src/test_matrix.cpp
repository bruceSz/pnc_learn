#include <Eigen/Eigen>
#include <iostream>

void diag(Eigen::VectorXd vec, int step) {
    Eigen::MatrixXd m = vec.asDiagonal();
    int dir = 0;
    if (step > 0)
        dir  = 1;
    else 
        dir = -1;

    Eigen::MatrixXd old = m;
    for (int i=0; i != step; i+=dir) {
        //int row = m.rows();
        Eigen::MatrixXd tmp(m.rows() + 1, m.cols());
        int col = m.cols();
        Eigen::MatrixXd z_line;
        z_line.setZero(1,col);
        if (dir) {
            tmp << m, z_line;
        } else {
            tmp << z_line, m;
        }

        Eigen::MetrixXd tmp2(tmp.rows(), tmp.cols() + 1);

        Eigen::MatrixXd z_col;
        z_col.setZero(tmp.rows(), 1);

        if (dir) {
            tmp2 << z_col, tmp2;
        } else {
            tmp2 << tmp2, z_col;
        }
        
    }
}


int main() {
    
    Eigen::Matrix<double, 4, 4> matrix_44;
     
    matrix_44 <<
            0, 0, 0, 0,
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0;

    Eigen::VectorXd d(3);
    d.setOnes();
    //d(0) = 1.;
   // d(1) = 2.;
    //d(2) = 3.;

    Eigen::MatrixXd m2 = d.asDiagonal();


     
    Eigen::MatrixXd m3(m2.rows()  +1, m2.cols() );
    const int cols_n = m2.cols();
    Eigen::MatrixXd zeros;

    zeros.setZero(1, m2.cols());
    std::cout << "m2: \n" <<  m2 << std::endl;
    std::cout << "tmp zero 1,cols matrix is: " << zeros << std::endl;
    //zeros.setZeros();
    m3 << m2, zeros ;
 
    std::cout << matrix_44 << std::endl;

    
    std::cout << m2.rows() << " : " << m2.cols() << std::endl;
    std::cout << "after merge on first line with 0." << std::endl;
    std::cout << m3 << std::endl;

}