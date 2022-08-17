#include "CubicSpline.hpp"

void PrintMatrix(Eigen::MatrixXd mat)
{
    for(int i = 0 ; i < mat.rows() ; i++)
    {
        for(int j = 0 ; j < mat.cols() ; j++)
        {
            std::cout << mat(i, j) << "  ";
        }
        std::cout << std::endl;
    }
}

/* constructor */
CubicSpline::CubicSpline()
{
    std::cout << "CubicSpline Birth Done" << std::endl;
}

/* destructor */
CubicSpline::~CubicSpline()
{
    std::cout << "CubicSpline Die ..." << std::endl;
}

void CubicSpline::CubicSplineRun(Eigen::MatrixXd P_)
{
    /* error checking */
    if((P_.rows()) != 2)
        std::cout << "P is in wrong rows !!!" << std::endl;

    num_control_pt = P_.cols(); 
    N = num_control_pt - 1;
    a.resize(N);
    b.resize(N);
    c.resize(N);
    d.resize(N);

    a = P_.block(1, 0, 1, N).transpose();
    #ifdef CUBICSPLINE_TEST_PRINT_A
        std::cout << "----------------------------------------- C -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << a[i] << std::endl;
    #endif
}