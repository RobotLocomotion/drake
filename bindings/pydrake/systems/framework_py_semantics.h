#pragma once

/// @file
/// Provides the semantics portion of Python bindings for the systems
/// framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

void DefineFrameworkPySemantics(py::module m);

// The DiagramBuilder really should be part of framework_py_systems.cc but for
// historical reasons is part of framework_py_semantics.cc. We'll provide a
// standalone function to bind it so framework_py.cc can sequence it properly.
void DefineFrameworkDiagramBuilder(py::module m);

}  // namespace pydrake
}  // namespace drake
