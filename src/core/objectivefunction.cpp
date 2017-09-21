#include "objectivefunction.h"

ObjectiveFunction::ObjectiveFunction()
{

}

ObjectiveFunction::~ObjectiveFunction()
{

}

Eigen::SparseMatrix<double> ObjectiveFunction::hessian(const Eigen::VectorXd &x)
{
    Eigen::SparseMatrix<double> H;
    uint n = x.size();
    H.resize(n,n);
    H.resizeNonZeros(n);
    H.setIdentity();
}
