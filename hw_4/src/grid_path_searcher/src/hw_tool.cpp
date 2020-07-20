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
    double optimal_cost = 100000.0; 
    double delta_p_x = _target_position(0) - _start_position(0);
    double delta_p_y = _target_position(1) - _start_position(1);
    double delta_p_z = _target_position(2) - _start_position(2);
    
    double delta_p_x_square = delta_p_x * delta_p_x;
    double delta_p_y_square = delta_p_y * delta_p_y;
    double delta_p_z_square = delta_p_z * delta_p_z;

    double delta_v_x = 0 - _start_velocity(0);
    double delta_v_y = 0 - _start_velocity(1);
    double delta_v_z = 0 - _start_velocity(2);

    double delta_v_x_square = delta_v_x * delta_v_x;
    double delta_v_y_square = delta_v_y * delta_v_y;
    double delta_v_z_square = delta_v_z * delta_v_z;


    double vx0 = _start_velocity(0);
    double vy0 = _start_velocity(1);
    double vz0 = _start_velocity(2);

    
    double a = delta_p_x_square  + delta_p_y_square + delta_p_z_square;
    double b = delta_v_x_square  + delta_v_y_square + delta_v_z_square;
    double c = vx0 * delta_p_x + vy0 * delta_p_y + vz0 * delta_p_z;
    
    std::cout << " a is: " << a << " b is : " << b << " c is: " << c << std::endl;
    
    
    vector<double> tmpOptimalCost(4, 100000.0);

    std::vector<double> coefs;

    auto coef_4 = 1.0f;
    auto coef_3 = 0.0;
    auto coef_2 = - 4 * b;
    auto coef_1 = +24 * c;
    auto coef_0 = -36 * a;

    coefs.push_back(coef_4);
    coefs.push_back(coef_3);
    coefs.push_back(coef_2);
    coefs.push_back(coef_1);
    coefs.push_back(coef_0);

    std::vector<double> T_vec = computeRoot(coefs);
    
    for (auto T : T_vec) {
        double T2 = std::pow(T, -2);
        double T3 =  std::pow(T, -3);
        double tmp  = T + (12*a) * T3 - (12*c) * T2 + (4*b)/T;

        tmpOptimalCost.push_back(tmp);
    }

    for(auto c: tmpOptimalCost)
    {
        optimal_cost = std::min(c,optimal_cost);
    }
	
    return optimal_cost;
}




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


double computeRoot4(double a, double b, double c, double d, double e) {
    // a is the coef of T^4;
    // 

    Eigen::Matrix<double, 4, 4> CompanionMatrix44;
    CompanionMatrix44 << 0, 0, 0, -e,
				         1, 0, 0, -d,
				         0, 1, 0, -c,
				         0, 0, 1, 0 ;


    std::cout << " a is: " << a
        << " b is: " << b
        << " c is: " << c
        << " d is: " << d
        << " e is: " << e << std::endl;


    auto eigenVals = CompanionMatrix44.eigenvalues();
    double res = 0;
    for(int i=0;i<eigenVals.size(); i++) {
        auto img = eigenVals(i).imag();
        auto real = eigenVals(i).real();
        std::cout << "real is : " << real << " img is : " << img << std::endl;
        if (std::abs(img) < 1e-16 && real > 0.0 )   {
            res = real;
            std::cout << "res is " << res << std::endl;
        }
        
    }

    return res;

}

// refer: https://blog.csdn.net/fb_941219/article/details/102991181
std::vector<double> computeRoot(std::vector<double> coefs ) {
    //double optimal_cost = 100000.;

    int total = coefs.size();
    //coefs[0] is the highest order of the poly.
    std::vector<double> real_coefs = std::vector<double>(coefs.begin() + 1, coefs.end() );
    int r_coefs_size = real_coefs.size();


    std::vector<double> new_vec;

    std::cout << "first coef is i: " << 0 << " val: " << coefs[0] << std::endl;
    new_vec.resize(real_coefs.size());
    for(int i =0;i<real_coefs.size(); i++) {
        std::cout << " coef i: "  << i+1 << "  val: " << real_coefs[i] << std::endl;
        double tmp = -1 * real_coefs[i]/ coefs[0];
        new_vec[i] = tmp;
    }



    Eigen::VectorXd diag_ones;
    diag_ones.setOnes( total-2);
    //diag_ones.setOnes(1, total-2);
    

    Eigen::MatrixXd res =  diag(diag_ones, -1);

    //std::cout << "After create diagonal." 
    //    << " the res is:" << res
    //    << std::endl;
    
    assert(res.cols()== new_vec.size());
    for(int i=0;i<res.cols(); i++) {
        res(0,i) = new_vec[i];
    }

    std::vector<double> ret;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> matrix_eigenvalues = res.eigenvalues();
    for(int i =0; i< matrix_eigenvalues.rows(); i++ ) {
        double real = std::real(matrix_eigenvalues(i));
        double img = std::imag(matrix_eigenvalues(i));
        
        std::cout << "real: " << real << "; img: " << img << std::endl;
        if((real <= 0) || std::abs(img) >= 1e-16){ 
            // ignoring negative roots and complex roots, if all roots are complex, the function J is monotonous
            // 
            continue;
        }
        //std::cout << " solution " << i << " real: " << real << " img: " << img << std::endl;
        ret.push_back(real);
        
        std::cout << " valid root is " << real << std::endl;        
    }

   return ret;

}