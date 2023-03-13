#pragma once

/* This file declares the functions that bind the drake::manipulation namespace.
These functions form a complete partition of the drake::manipulation bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per manipulation_py_kuka_iiwa.cc. */
void DefineManipulationKukaIiwa(py::module m);

/* Defines bindings per manipulation_py_schunk_wsg.cc. */
void DefineManipulationSchunkWsg(py::module m);

/* Defines bindings per manipulation_py_util.cc. */
void DefineManipulationUtil(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
