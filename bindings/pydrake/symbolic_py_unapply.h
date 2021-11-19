#pragma once

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

constexpr char kUnapplyExpressionDoc[] = R"""(
Given an expression, returns a tuple (ctor, args) that would re-create an
equivalent expression when called as ctor(*args).  This is a useful way to
unpack the contents of a compound expression, e.g., to obtain the terms of
an addition.  In many cases (all arithmetic operators, trig functions,
max, min, sqrt, etc.), the returned args will all be either Expressions or
floats; in other cases (e.g., if_then_else) some args will other types
(e.g., a Formula).  To check the form (i.e., kind) of an expression, use
e.get_kind(); do not try to infer it from the returned ctor's identity.
)""";

// @param m The pydrake.symbolic module.
py::object Unapply(py::module m, const symbolic::Expression& e);

constexpr char kUnapplyFormulaDoc[] = R"""(
Given a formula, returns a tuple (ctor, args) that would re-create an
equivalent formula when called as ctor(*args).  This is a useful way to
unpack the contents of a compound formula, e.g., to obtain the terms of
a comparison.  For relational formulae (==, <, >, etc.) the returned args
will both be of type Expression.  For compound formulae (and, or, not) the
returned args will be of type Formula.  To check the form (i.e., kind) of
a formula, use f.get_kind(); do not try to infer it from he returned
ctor's identity.
)""";

// @param m The pydrake.symbolic module.
py::object Unapply(py::module m, const symbolic::Formula& f);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
