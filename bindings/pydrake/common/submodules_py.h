#pragma once

/* This file declares the functions that populate the sub-modules of
`pydrake.common`. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

// For simplicity, these declarations are listed in alphabetical order.

/* Defines `pydrake.common.eigen_geometry` bindings per eigen_geometry_py.cc. */
void DefineModuleEigenGeometry(py::module m);

/* Defines `pydrake.common.schema` bindings per schema_py.cc. */
void DefineModuleSchema(py::module m);

/* Defines `pydrake.common.value` bindings per value_py.cc. */
void DefineModuleValue(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
