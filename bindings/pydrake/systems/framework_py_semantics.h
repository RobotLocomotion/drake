#pragma once

/// @file
/// Provides the semantics portion of Python bindings for the systems
/// framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

void DefineFrameworkPySemantics(py::module m);

}  // namespace pydrake
}  // namespace drake
