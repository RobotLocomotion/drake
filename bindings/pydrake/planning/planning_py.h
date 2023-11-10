#pragma once

/* This file declares the functions that bind the drake::planning namespace.
These functions form a complete partition of the drake::planning bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

// For simplicity, these declarations are listed in alphabetical order.

/* Defines bindings per planning_py_collision_checker.cc. */
void DefinePlanningCollisionChecker(py::module m);

/* Defines bindings per planning_py_collision_checker_interface_types.cc. */
void DefinePlanningCollisionCheckerInterfaceTypes(py::module m);

/* Defines bindings per planning_py_graph_algorithms.cc. */
void DefinePlanningGraphAlgorithms(py::module m);

/* Defines bindings per planning_py_robot_diagram.cc. */
void DefinePlanningRobotDiagram(py::module m);

/* Defines bindings per planning_py_trajectory_optimization.cc. */
void DefinePlanningTrajectoryOptimization(py::module m);

/* Defines bindings per planning_py_visibility_graph.cc. */
void DefinePlanningVisibilityGraph(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
