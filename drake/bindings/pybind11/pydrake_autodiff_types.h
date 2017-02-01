#include <pybind11/pybind11.h>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

typedef Eigen::AutoDiffScalar<Eigen::VectorXd> AutoDiffXd;

typedef Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1> VectorXAutoDiffXd;
PYBIND11_MAKE_OPAQUE(VectorXAutoDiffXd);

typedef Eigen::Matrix<AutoDiffXd, 3, Eigen::Dynamic> Matrix3XAutoDiffXd;
PYBIND11_MAKE_OPAQUE(Matrix3XAutoDiffXd);

typedef Eigen::Matrix<AutoDiffXd, 4, 4> Matrix44AutoDiffXd;
PYBIND11_MAKE_OPAQUE(Matrix44AutoDiffXd);
