#include "CubicSpline.hpp"

int main()
{
    CubicSpline* CS = new CubicSpline;
    
    Eigen::MatrixXd control_pt;
    control_pt.resize(2, 4);
    control_pt.col(0) << 0.0, 0.0;
    control_pt.col(1) << 2.0, 6.0;
    control_pt.col(2) << 3.0, -1.0;
    control_pt.col(3) << 7.0, 0.0;

    CS->CubicSplineRun(control_pt);

    delete CS;
    return 0;
}