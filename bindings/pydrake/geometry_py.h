#pragma once

/** @file This files declares the functions which bind various portions of the
 geometry namespace. These functions form a complete partition of all geometry
 bindings. The implementation of these methods are parceled out into
 various .cc files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/** Defines all elements in the drake::geometry::optimization namespace.
 See geometry_py_optimization.cc. */
void DefineGeometryOptimization(py::module m);

}  // namespace pydrake
}  // namespace drake
