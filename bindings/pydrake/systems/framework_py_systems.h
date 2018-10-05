#pragma once

/// @file
/// Provides the Python bindings for the core system classes of the systems
/// framework.

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

void DefineFrameworkPySystems(py::module m);

}  // namespace pydrake
}  // namespace drake
