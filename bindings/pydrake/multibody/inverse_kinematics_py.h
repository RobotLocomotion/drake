#pragma once

/* This file declares the functions that bind into the
 drake.multibody.inverse_kinematics module. These functions form a complete
 partition of the drake.multibody.inverse_kinematics bindings. Note: the
 artifacts all belong in the drake::multibody namespace.

 The implementations of these functions are parceled out into various *.cc
 files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per inverse_kinematics_py_differential.cc. */
void DefineIkDifferential(py::module m);

// TODO(SeanCurtis-TRI): Make the partition promise true by refactoring
// inverse_kinematics_py.cc into its own function declared here.

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
