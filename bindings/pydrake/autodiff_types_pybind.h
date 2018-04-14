#pragma once

#include <Eigen/Core>
#include "pybind11/numpy_dtypes_user.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/autodiff.h"

// The macro `PYBIND11_NUMPY_DTYPE_USER` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

PYBIND11_NUMPY_DTYPE_USER(drake::AutoDiffXd);
