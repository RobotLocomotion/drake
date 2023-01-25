#pragma once

/* This file declares the functions that bind the drake::planning namespace.
These functions form a complete partition of the drake::planning bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per planning_py_robot_diagram.cc. */
void DefinePlanningRobotDiagram(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
