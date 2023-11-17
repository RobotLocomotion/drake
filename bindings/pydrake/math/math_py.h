#pragma once

/* This file declares the functions that bind the drake::math namespace.
These functions form a complete partition of the drake::math bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation.

TODO(jwnimmer-tri) At the moment there is no parceling, just one obnoxiously
large file. We should work to split it up. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Binds a native C++ matmul(A, B) for wide variety of scalar types. This is
important for performance until numpy allows us to define dtype-specific
implementations. Doing a matmul elementwise with a C++ <=> Python call for every
flop is extraordinarily slow. */
void DefineMathMatmul(py::module m);

/* Defines bindings per math_py_monolith.cc. */
void DefineMathMonolith(py::module m);

/* Defines bindings per math_py_operators.cc. */
void DefineMathOperators(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
