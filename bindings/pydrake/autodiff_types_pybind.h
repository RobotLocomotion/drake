#pragma once

#include <Eigen/Core>
#include <pybind11/pybind11.h>

#include "drake/common/autodiff.h"

PYBIND11_NUMPY_OBJECT_DTYPE(drake::AutoDiffXd);

typedef Eigen::Matrix<drake::AutoDiffXd, Eigen::Dynamic, 1> VectorXAutoDiffXd;

typedef Eigen::Matrix<drake::AutoDiffXd, 3, Eigen::Dynamic> Matrix3XAutoDiffXd;

typedef Eigen::Matrix<drake::AutoDiffXd, 4, 4> Matrix44AutoDiffXd;
