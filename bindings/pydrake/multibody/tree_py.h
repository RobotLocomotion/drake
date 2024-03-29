#pragma once

/* This file declares some functions that help bind into the
 drake.multibody.tree module.

 The implementations of these functions are parceled out into various *.cc
 files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per tree_py_inertia.cc. */
void DefineTreeInertia(py::module m);

// TODO(jwnimmer-tri) Add more functions here as we continue to split up the
// tree_py.cc file into difference pieces.

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
