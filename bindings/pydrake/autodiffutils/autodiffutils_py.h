#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines all bindings for the pydrake.autodiffutils module. */
void DefineAutodiffutils(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
