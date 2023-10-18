#pragma once

/* This file declares the functions that bind the drake::symbolic namespace.
These functions form a complete partition of the drake::symbolic bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation.

TODO(jwnimmer-tri) At the moment there is no parceling, just one obnoxiously
large file. We should work to split it up. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per symbolic_py_monolith.cc. */
void DefineSymbolicMonolith(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
