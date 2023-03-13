#pragma once

#include "pybind11/eigen.h"
#include "pybind11/numpy.h"

#include "drake/common/autodiff.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

PYBIND11_NUMPY_OBJECT_DTYPE(drake::AutoDiffXd);
