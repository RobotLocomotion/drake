#pragma once

/* This file declares the functions that bind the
 drake::multibody::inverse_kinematics namespace. These functions form a complete
 partition of the drake::multibody::inverse_kinematics bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per inverse_kinematics_py_differential.cc. */
void DefineIkDifferential(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
