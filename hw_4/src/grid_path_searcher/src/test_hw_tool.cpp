/**
 * Readme!
 * g++ hw_tool.cpp -I/usr/include/eigen3 -I/usr/local/include/eigen3 -std=c++11 -o hw4_test && ./hw4_test
**/
#include <iostream>
#include <assert.h>
#include <vector>
#include <unsupported/Eigen/Polynomials>

#include "hw_tool.h"


using namespace std;
using namespace Eigen;





// optimal T is a positive root
// root 0 is: (1.81105,3.50733)
// root 1 is: (1.81105,-3.50733)
// root 2 is: (-6.18502,0)
// root 3 is: (2.56292,0)
// optimal_cost : 4.01138
int main(int argc, char const *argv[])
{

    
    //Homeworktool *     = new Homeworktool();

    Eigen::Vector3d p0, v0, pt;
    p0 << -0.557813, 1.06781, 0.0812813;
    v0 << -1.	   ,   1.475,    0.1275;
    pt << -2.30443 , 3.01808,         0;

    Homeworktool hw4;
    double cost = hw4.OptimalBVP(p0, v0, pt);
    cout << "optimal_cost : " << cost << endl;
    assert(fabs(cost - 4.01138) < 0.001);

    return 0;
}