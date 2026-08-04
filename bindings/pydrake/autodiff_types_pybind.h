#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/autodiff.h"

PYDRAKE_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::AutoDiffXd)
