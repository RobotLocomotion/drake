#pragma once

#include <string>

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* pydrake::internal::ref_cycle is a custom call policy for pybind11.

  For an overview of other call policies, See
  https://pybind11.readthedocs.io/en/stable/advanced/functions.html#additional-call-policies

  `ref_cycle` creates a reference count cycle that Python's cyclic garbage
  collection can find and collect, once the cycle's objects are no longer
  reachable.

  Both peer objects must be either ordinary python objects or pybind11 classes
  that were defined with the dynamic_attr() annotation. If either of the peer
  objects is a pybind11 class not defined with py::dynamic_attr(), the
  binding will raise an exception at the time the function or method is
  called. The error message will complain about objects not being tracked by
  garbage collection. By default pybind11 python objects are not tracked; the
  dynamic_attr() annotation makes sure the objects are tracked. Also, it
  ensures that the implementation of `ref_cycle` can create and assign a new
  attribute dynamically, which is how the garbage collectible bookkeeping is
  accomplished.

  `ref_cycle` causes each object to refer to the other in a cycle. It is
  bidirectional and symmetric. The order of the template arguments does not
  matter.

  Note the consequences for object lifetimes:

  * M keeps N alive, and N keeps M alive.
  * Neither object is destroyed until:
    * both are unreachable, and
    * garbage collection runs.

  @tparam Peer0 an argument index
  @tparam Peer1 an argument index

  The argument index starts at 1; for methods, `self` is at index 1. Index 0
  denotes the return value.
 */
template <size_t Peer0, size_t Peer1>
struct ref_cycle {};

/* This function is used in the template below to select peers by call/return
 index. */
void ref_cycle_impl(
    size_t peer0, size_t peer1, PyObject** args, size_t nargs, py::handle ret);

/* This function constructs a reference cycle from arbitrary handles. It may be
 needed in special cases where the ordinary call-policy annotations won't work.
 The `location_hint` will appear in any exception messages; it should help
 developers locate where and why this function was called. */
void make_arbitrary_ref_cycle(
    py::handle p0, py::handle p1, const std::string& location_hint);

/* This function constructs a one-way reference from handle p0 to p1. It may be
 needed in special cases where the ordinary call-policy annotations won't work.
 The `location_hint` will appear in any exception messages; it should help
 developers locate where and why this function was called. */
void make_arbitrary_ref_link(
    py::handle p0, py::handle p1, const std::string& location_hint);

// TODO(rpoyner-tri): figure out if this feature can capture the function name
// for use in error messages.

// Returns true if either template parameter denotes the return value.
template <size_t Peer0, size_t Peer1>
static constexpr bool needs_return_value() {
  return Peer0 == 0 || Peer1 == 0;
}

template <size_t NArgs, size_t Peer0, size_t Peer1>
NB_INLINE void process_precall(PyObject** args,
    std::integral_constant<size_t, NArgs>, nanobind::detail::cleanup_list*,
    ref_cycle<Peer0, Peer1>*) {
  if constexpr (!needs_return_value<Peer0, Peer1>()) {
    ref_cycle_impl(Peer0, Peer1, args, NArgs, nanobind::handle());
  }
}

template <size_t NArgs, size_t Peer0, size_t Peer1>
void process_postcall(PyObject** args, std::integral_constant<size_t, NArgs>,
    PyObject* result, ref_cycle<Peer0, Peer1>*) {
  if constexpr (drake::pydrake::internal::needs_return_value<Peer0, Peer1>()) {
    // result_guard avoids leaking a reference to the return object if postcall
    // throws an exception.
    py::object result_guard = py::steal(result);
    ref_cycle_impl(Peer0, Peer1, args, NArgs, result);
    result_guard.release();
  }
}

template <typename F, size_t Peer0, size_t Peer1>
NB_INLINE void func_extra_apply(F& f, ref_cycle<Peer0, Peer1>, size_t&) {
  f.flags |=
      static_cast<uint32_t>(nanobind::detail::func_flags::can_mutate_args);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace nanobind {
namespace detail {

// Provide specializations of the nanobind internal templates for call
// policies; this allows writing an annotation that works seamlessly in
// bindings definitions.

template <size_t Peer0, size_t Peer1, typename... Ts>
struct func_extra_info<drake::pydrake::internal::ref_cycle<Peer0, Peer1>, Ts...>
    : func_extra_info<Ts...> {
  static constexpr bool pre_post_hooks = true;
};

}  // namespace detail
}  // namespace nanobind
