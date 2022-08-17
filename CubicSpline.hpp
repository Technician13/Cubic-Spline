#ifndef CUBICSPLINEHPP
#define CUBICSPLINEHPP

#include <iostream>
#include <Eigen/Dense>

/* ************************************ option start ************************************ */
/* print a */
// #define CUBICSPLINE_TEST_PRINT_A
/* ************************************ option end ************************************ */

class CubicSpline
{
    private:
        /* number of control point */
        int num_control_pt;
        /* number of segment */
        int N;
        /* set of parameters a_i */
        Eigen::VectorXd a;
        /* set of parameters b_i */
        Eigen::VectorXd b;
        /* set of parameters c_i */
        Eigen::VectorXd c;
        /* set of parameters d_i */
        Eigen::VectorXd d;
        
    protected:
        
    public:
        CubicSpline();
        ~CubicSpline();
        void CubicSplineRun(Eigen::MatrixXd P_);
};

#endif