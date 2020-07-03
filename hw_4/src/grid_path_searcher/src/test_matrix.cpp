#include <Eigen/Eigen>
#include <iostream>
#include <cassert>

Eigen::MatrixXd diag(Eigen::VectorXd vec, int total_step) {
    Eigen::MatrixXd m = vec.asDiagonal();

    //assert(step < vec.size() -1);
    bool ul;
    if (total_step > 0)
        ul = true ;
    else 
        ul = false;

    Eigen::MatrixXd old = m;
    uint32_t abs_total = abs(total_step);
    for (int i=0; i < abs_total; i++) {
        
        std::cout << "Entering the tmp." << std::endl;
        Eigen::MatrixXd tmp(old.rows() + 1, old.cols());
        int col = old.cols();

        Eigen::MatrixXd z_line;
        z_line.setZero(1,col);
        if (ul) {
            tmp << old, z_line;
        } else {
            tmp << z_line, old;
        }

        std::cout<< " new row added." << std::endl;

        Eigen::MatrixXd tmp2(tmp.rows(), tmp.cols() + 1);

        Eigen::MatrixXd z_col;
        z_col.setZero(tmp.rows(), 1);

        if (ul) {
            tmp2 << z_col, tmp;
        } else {
            tmp2 << tmp, z_col;
        }
        std::cout << " new cols added." << std::endl;

        old = tmp2;
        std::cout << "assign the second time" << std::endl;
        
    }
    return old;
}

int main1() {

    Eigen::VectorXd d(3);
    d.setOnes();
    d(0) = 1.;
    d(1) = 2.;
    d(2) = 3.; 
    Eigen::MatrixXd out = diag(d,0);
    std::cout << " out is :\n" << out << std::endl;

    out = diag(d, 1);
    std::cout << " out ul is :\n" << out << std::endl;

    out = diag(d, -1);
    std::cout << " out dr is :\n" << out << std::endl;

    out = diag(d, 2);
    std::cout << " out ul 2 is :\n" << out << std::endl;

    out = diag(d, -2);
    std::cout << " out dr 2 is :\n" << out << std::endl;


}


double computeRoot(std::vector<double> coefs ) {

    int total = coefs.size();
    //coefs[0] is the highest order of the poly.
    std::vector<double> real_coefs = std::vector<double>(coefs.begin() + 1, coefs.end() );
    int r_coefs_size = real_coefs.size();


    std::vector<double> new_vec;
    new_vec.resize(real_coefs.size());
    for(int i =0;i<real_coefs.size(); i++) {
        double tmp = -1 * real_coefs[i]/ coefs[0];
        new_vec[i] = tmp;
    }


    Eigen::VectorXd diag_ones;
    diag_ones.setOnes( total-2);
    //diag_ones.setOnes(1, total-2);
    

    Eigen::MatrixXd res =  diag(diag_ones, -1);

    std::cout << "After create diagonal." 
        << " the res is:" << res
        << std::endl;
    
    assert(res.cols()== new_vec.size());
    for(int i=0;i<res.cols(); i++) {
        res(0,i) = new_vec[i];
    }

    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> matrix_eigenvalues = res.eigenvalues();
    for(int i =0; i< matrix_eigenvalues.rows(); i++ ) {
        double real = std::real(matrix_eigenvalues(i));
        double img = std::imag(matrix_eigenvalues(i));
        std::cout << " solution " << i << " real: " << real << " img: " << img << std::endl;
    }



}


int main() {
    
    Eigen::Matrix<double, 4, 4> matrix_44;
     
    matrix_44 <<
            0, 0, 0, 0,
            1, 0, 0, 0,
            0, 1, 3, 0,
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

    Eigen::MatrixXd m4(m3.rows(), m3.cols() + 1);

    Eigen::MatrixXd z2;
    z2.setZero(m3.rows(),1);

    m4 << z2 , m3 ;
    std::cout << "The total is \n" << m4 << std::endl;


    std::vector<double> p ;
    p.push_back(10);
    p.push_back(0 );
    p.push_back(0 );
    p.push_back(3 );
    p.push_back(2 );

    computeRoot(p);
    //Eigen::VectorXd p(5);
    // 10 0 0 3 2 
    //p(0) = 10;
    //p(1) = 0;
    //p(2) = 0;
    //p(3) = 3;
    //p(4) = 2;





}