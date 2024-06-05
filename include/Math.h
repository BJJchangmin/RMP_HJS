#include "globals.h"

#include<eigen-master/Eigen/Core>
#include<eigen-master/Eigen/Dense>

using namespace Eigen;

MatrixXd pinv(const MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}
