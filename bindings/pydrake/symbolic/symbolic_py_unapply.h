#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
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

// Given the pydrake.symbolic module as "m" and an expression "kind", returns
// the callable object (i.e., factory function or constructor) that would be
// able to re-construct the same expression, given appropriate arguments. This
// is the "ctor" returned by Unapply() for the "e.get_kind()".
py::object MakeUnapplyConstructor(py::module m, symbolic::ExpressionKind kind);

// Given the pydrake.symbolic module as "m" and a formula "kind", returns the
// callable object (i.e., factory function or constructor) that would be able to
// re-construct the same formula, given appropriate arguments. This is the
// "ctor" returned by Unapply() for the "f.get_kind()".
py::object MakeUnapplyConstructor(py::module m, symbolic::FormulaKind kind);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
