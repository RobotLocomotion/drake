#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<double>)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::AutoDiffXd>)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::symbolic::Expression>)
