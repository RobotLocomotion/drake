#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/** Defines all elements in the drake::geometry::optimization namespace.
 See geometry_py_optimization.cc. */
void DefineGeometryOptimization(py::module m);

}  // namespace pydrake
}  // namespace drake
