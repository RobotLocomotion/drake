#include <pybind11/pybind11.h>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AutoDiffXd;
PYBIND11_NUMPY_OBJECT_DTYPE(AutoDiffXd);

typedef Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1> VectorXAutoDiffXd;

typedef Eigen::Matrix<AutoDiffXd, 3, Eigen::Dynamic> Matrix3XAutoDiffXd;

typedef Eigen::Matrix<AutoDiffXd, 4, 4> Matrix44AutoDiffXd;
