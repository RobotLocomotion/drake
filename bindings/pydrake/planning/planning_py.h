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

/* Defines bindings per planning_py_dof_mask.cc. */
void DefinePlanningDofMask(py::module m);

/* Defines bindings per planning_py_graph_algorithms.cc. */
void DefinePlanningGraphAlgorithms(py::module m);

/* Defines bindings per planning_py_iris_from_clique_cover.cc. */
void DefinePlanningIrisFromCliqueCover(py::module m);

/* Defines bindings per planning_py_iris_common.cc. */
void DefinePlanningIrisCommon(py::module m);

/* Defines bindings per planning_py_iris_np2.cc. */
void DefinePlanningIrisNp2(py::module m);

/* Defines bindings per planning_py_iris_zo.cc. */
void DefinePlanningIrisZo(py::module m);

/* Defines bindings per planning_py_joint_limits.cc. */
void DefinePlanningJointLimits(py::module m);

/* Defines bindings per planning_py_robot_diagram.cc. */
void DefinePlanningRobotDiagram(py::module m);

/* Defines bindings per planning_py_trajectory_optimization.cc. */
void DefinePlanningTrajectoryOptimization(py::module m);

/* Defines bindings per planning_py_visibility_graph.cc. */
void DefinePlanningVisibilityGraph(py::module m);

/* Defines bindings per planning_py_zmp_planner.cc. */
void DefinePlanningZmpPlanner(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
