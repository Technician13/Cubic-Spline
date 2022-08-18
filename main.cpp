#include "CubicSpline.hpp"

int main()
{
    CubicSpline* CS = new CubicSpline;
    
    Eigen::MatrixXd control_pt;
    control_pt.resize(2, 4);
    control_pt.col(0) << 0.0, 0.0;
    control_pt.col(1) << 2.0, 6.0;
    control_pt.col(2) << 3.0, 1.0;
    control_pt.col(3) << 10.0, 3.0;

    CS->CubicSplineRun(control_pt, NATURAL);

    /* Tips:
     *  The coefficients of each segment of the whole path
     *  are loaded in CS->a, CS->b, CS->c and CS->d after run
     *  the function CS->CubicSplineRun, if you want to know
     *  the specific form of the equation corresponding to
     *  each paragraph, please do not commented macro
     *  definition CUBICSPLINE_TEST_PRINT_EQUATION. */

    delete CS;
    return 0;
}