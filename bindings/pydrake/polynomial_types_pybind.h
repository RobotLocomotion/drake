#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

PYDRAKE_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<double>)
PYDRAKE_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::AutoDiffXd>)
PYDRAKE_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::Polynomial<drake::symbolic::Expression>)
