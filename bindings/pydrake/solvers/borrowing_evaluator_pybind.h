#pragma once

/// @file
/// Helpers for bindings that hold on to a Binding<> whose evaluator might
/// borrow Python callables (e.g., a Cost made from a Python function).

#include <memory>
#include <utility>

#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/binding.h"

namespace drake {
namespace pydrake {
namespace internal {

// An evaluator "borrows" Python callables when its bindings keep those
// callables alive with ref links, so that garbage collection can see any cycle
// they form. That makes the evaluator's Python object the source of a ref link;
// see is_ref_link_source(). This relies on a convention: every evaluator that
// borrows callables must register a link, and none should register one
// otherwise.
//
// This is the one authoritative definition of "borrowing". It must be asked
// after the evaluator's links are made.
//
// A borrowing evaluator's C++ object is owned by its Python object (or is
// stored within it), so holding the Python object also keeps the C++ object and
// its callables alive. A non-borrowing evaluator holds no Python references of
// its own, so C++ shared_ptr ownership of it is already enough.
inline bool EvaluatorBorrowsPythonCallables(py::handle evaluator_py) {
  return is_ref_link_source(evaluator_py);
}

// Returns a Binding that is safe to keep in C++ storage whose lifetime is
// unrelated to any Python object (e.g., a GraphOfConvexSets vertex).
//
// A binding whose evaluator borrows Python callables may refer to its evaluator
// without owning it, relying on some other Python object to keep that evaluator
// (and its callables) alive. If a binding like that is copied into a place that
// has no such Python object, the copy dangles. This function returns a binding
// whose shared_ptr owns the evaluator's Python object instead, so that the
// evaluator stays alive for as long as the returned binding or any copy of it
// does. (The cost is that a callable that refers back to the object holding the
// binding forms a cycle that garbage collection can't see. That's no worse than
// not borrowing at all.)
//
// Any other binding is returned unchanged.
template <typename C>
solvers::Binding<C> ReownBindingIfBorrowing(
    const solvers::Binding<C>& binding) {
  py::object evaluator_py = py::cast(binding.evaluator());
  if (!EvaluatorBorrowsPythonCallables(evaluator_py)) {
    return binding;
  }
  return solvers::Binding<C>(
      make_shared_ptr_from_py_object<C>(std::move(evaluator_py)),
      binding.variables());
}

// Wraps `method`, a member function of Self whose first argument is a
// solvers::Binding<C>, to call it with ReownBindingIfBorrowing(binding)
// instead. Use this to bind any method that stores its Binding argument.
//
// For example:
//   cls.def("AddCost",
//       ReownBinding(py::overload_cast<const Binding<Cost>&>(&Self::AddCost)),
//       py::arg("binding"));
template <typename Self, typename R, typename C, typename... Rest>
auto ReownBinding(R (Self::*method)(const solvers::Binding<C>&, Rest...)) {
  return [method](Self& self, const solvers::Binding<C>& binding,
             Rest... rest) -> R {
    return (self.*method)(
        ReownBindingIfBorrowing(binding), std::forward<Rest>(rest)...);
  };
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
