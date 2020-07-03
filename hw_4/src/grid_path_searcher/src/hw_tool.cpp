#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
                    



    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory


    */
    return optimal_cost;
}


Eigen::MatrixXd Homeworktool::diag(Eigen::VectorXd vec, int total_step) {
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

double Homeworktool::computeRoot(std::vector<double> coefs ) {

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