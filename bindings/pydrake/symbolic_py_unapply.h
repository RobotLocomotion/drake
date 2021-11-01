#pragma once

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

constexpr char kUnapplyDoc[] = R"""(
Given an expression, returns a tuple (ctor, args) that would re-create an
equivalent expression.  This is a useful way to unpack the contents of a
compound expression, e.g., to obtain the terms of an addition.
)""";

// @param m The pydrake.symbolic module.
py::object Unapply(py::module m, const symbolic::Expression& e);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
