#pragma once

#include "pybind11/eigen.h"
#include "pybind11/numpy.h"

#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/common/symbolic/rational_function.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

// Whenever we want to cast any array / matrix type of `T` in C++
// (e.g. `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the
// following list.
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Expression);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Formula);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Monomial);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Polynomial);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::RationalFunction);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Variable);
