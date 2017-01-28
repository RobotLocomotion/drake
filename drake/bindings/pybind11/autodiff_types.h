#include <pybind11/pybind11.h>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

typedef Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, Eigen::Dynamic, 1> VectorXAutoDiffXd;
PYBIND11_MAKE_OPAQUE(VectorXAutoDiffXd);

typedef Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, 3, Eigen::Dynamic> Matrix3XAutoDiffXd;
PYBIND11_MAKE_OPAQUE(Matrix3XAutoDiffXd);
