#ifndef CUBICSPLINEHPP
#define CUBICSPLINEHPP

#include <iostream>
#include <Eigen/Dense>

/* ************************************ option start ************************************ */
/* print h */
#define CUBICSPLINE_TEST_PRINT_H
/* print G */
#define CUBICSPLINE_TEST_PRINT_G
/* print r */
#define CUBICSPLINE_TEST_PRINT_R
/* print m */
#define CUBICSPLINE_TEST_PRINT_M
/* print a */
#define CUBICSPLINE_TEST_PRINT_A
/* print b */
#define CUBICSPLINE_TEST_PRINT_B
/* print c */
#define CUBICSPLINE_TEST_PRINT_C
/* print d */
#define CUBICSPLINE_TEST_PRINT_D
/* print equation */
#define CUBICSPLINE_TEST_PRINT_EQUATION
/* ************************************ option end ************************************ */

enum END_CONDITION {NATURAL, CLAMPED, NOT_A_KNOT};

class CubicSpline
{
    private:
        /* number of control point */
        int num_control_pt;
        /* number of segment */
        int N;
        /* set of step length h_i */
        Eigen::VectorXd h;
        /* set of parameters m_i */
        Eigen::VectorXd m;
        /* G * m = r */
        Eigen::MatrixXd G;
        Eigen::VectorXd r;
        
    protected:
        
    public:
        /* set of parameters a_i */
        Eigen::VectorXd a;
        /* set of parameters b_i */
        Eigen::VectorXd b;
        /* set of parameters c_i */
        Eigen::VectorXd c;
        /* set of parameters d_i */
        Eigen::VectorXd d;
        
        CubicSpline();
        ~CubicSpline();
        void CubicSplineRun(Eigen::MatrixXd P_, END_CONDITION end_condition_);
};

#endif