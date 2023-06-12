#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"

// Whenever we want to cast any array / matrix type of `T` in C++ (e.g.,
// `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the following
// list.
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Expression)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Formula)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Monomial)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Polynomial)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::RationalFunction)
DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(  // NOLINT
    drake::symbolic::Variable)
