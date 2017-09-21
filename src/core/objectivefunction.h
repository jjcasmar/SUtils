#ifndef OBJECTIVEFUNCTION_H
#define OBJECTIVEFUNCTION_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

class ObjectiveFunction
{
public:
    ObjectiveFunction();
    virtual ~ObjectiveFunction();

    virtual double value(const Eigen::VectorXd &x) = 0;
    virtual Eigen::VectorXd gradient(const Eigen::VectorXd &x) = 0;
    virtual Eigen::SparseMatrix<double> hessian(const Eigen::VectorXd &x);
};

#endif // OBJECTIVEFUNCTION_H
