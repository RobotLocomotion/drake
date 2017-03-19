#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"

PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Variable);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Expression);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Formula);
