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

void CubicSpline::CubicSplineRun(Eigen::MatrixXd P_, END_CONDITION end_condition_)
{
    /* error checking */
    if((P_.rows()) != 2)
        std::cout << "P is in wrong rows !!!" << std::endl;

    num_control_pt = P_.cols(); 
    N = num_control_pt - 1;

    h.resize(N);
    m.resize(N + 1);
    G.resize(N + 1, N + 1);
    r.resize(N + 1);
    a.resize(N);
    b.resize(N);
    c.resize(N);
    d.resize(N);

    /* load h ------------------------------------------------------------------------ */
    for(int i = 0 ; i < N ; i++)
    {
        h(i) = P_(0, i + 1) - P_(0, i);
    }

    /* load G ------------------------------------------------------------------------ */
    if(end_condition_ == NATURAL)
    {
        G(0, 0) = 1.0;
        G(N, N) = 1.0;
    }
    else if(end_condition_ == CLAMPED)
    {
        G(0, 0) = 2.0 * h(0);
        G(0, 1) = h(0);
        G(N, N) = 2.0 * h(N - 1);
        G(N, N -1) = h(N - 1);
        
    }
    else if(end_condition_ == NOT_A_KNOT)
    {
        G(0, 0) = -1.0 * h(1);
        G(0, 1) = h(0) + h(1);
        G(0, 2) = -1.0 * h(0);
        G(N, N) = -1.0 * h(N - 2);
        G(N, N - 1) = h(N - 2) + h(N - 1);
        G(N, N - 2) = -1.0 * h(N - 1);
    }
    for(int i = 0 ; i < (N - 1) ; i++)
    {
        G(i + 1, i) = h(i);
        G(i + 1, i + 1) = 2.0 * (h(i) + h(i + 1));
        G(i + 1, i + 2) = h(i + 1);
    }

    /* load r ------------------------------------------------------------------------ */
    r(0) = 0.0;
    r(N) = 0.0;
    for(int i = 1 ; i <= (N - 1) ; i++)
    {
        r(i) = 6.0 * (((P_(1, i + 1) - P_(1, i)) / (h(i))) - ((P_(1, i) - P_(1, i - 1)) / (h(i - 1))));
    }

    /* Gm = r ------------------------------------------------------------------------ */
    m = G.colPivHouseholderQr().solve(r);

    /* load a ------------------------------------------------------------------------ */
    a = P_.block(1, 0, 1, N).transpose();

    /* load b & c & d ---------------------------------------------------------------- */
    for(int i = 0 ; i < N ; i++)
    {
        b(i) = (P_(1, i + 1) - P_(1, i)) / h(i) - 0.5 * h(i) * m(i) - h(i) * (m(i + 1) - m(i)) / 6.0;
        c(i) = 0.5 * m(i);
        d(i) = (m(i + 1) - m(i)) / (6.0 * h(i));
    }

    #ifdef CUBICSPLINE_TEST_PRINT_H
        std::cout << "----------------------------------------- h -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << h[i] << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_G
        std::cout << "----------------------------------------- G -----------------------------------------" << std::endl;
        PrintMatrix(G);
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_R
        std::cout << "----------------------------------------- r -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N + 1 ; i++)
            std::cout << r(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_M
        std::cout << "----------------------------------------- m -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N + 1 ; i++)
            std::cout << m(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_A
        std::cout << "----------------------------------------- a -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << a(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_B
        std::cout << "----------------------------------------- b -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << b(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_C
        std::cout << "----------------------------------------- c -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << c(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_D
        std::cout << "----------------------------------------- d -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
            std::cout << d(i) << std::endl;
    #endif
    #ifdef CUBICSPLINE_TEST_PRINT_EQUATION
        std::cout << "----------------------------------------- equation -----------------------------------------" << std::endl;
        for(int i = 0 ; i < N ; i++)
        {
            std::cout << "#" << (i+1) << ":    "
            << a(i) << " + " 
            << b(i) << " * ( x - " << P_(0, i) << " ) + "
            << c(i) << " * ( x - " << P_(0, i) << " )^2 + "
            << d(i) << " * ( x - " << P_(0, i) << " )^3" << std::endl;  
        }
        std::cout << std::endl;
    #endif
}