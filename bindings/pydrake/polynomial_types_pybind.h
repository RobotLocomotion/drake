#pragma once

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` places symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

PYBIND11_NUMPY_OBJECT_DTYPE(drake::Polynomial<double>);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::Polynomial<drake::AutoDiffXd>);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::Polynomial<drake::symbolic::Expression>);
