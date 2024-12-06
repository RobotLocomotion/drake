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
void ref_cycle_impl(size_t peer0, size_t peer1,
    const py::detail::function_call& call, py::handle ret);

/* This function constructs a reference cycle from arbitrary handles. It may be
 needed in special cases where the ordinary call-policy annotations won't work.
 The `location_hint` will appear in any exception messages; it should help
 developers locate where and why this function was called. */
void make_arbitrary_ref_cycle(
    py::handle p0, py::handle p1, const std::string& location_hint);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {

// Provide a specialization of the pybind11 internal process_attribute
// template; this allows writing an annotation that works seamlessly in
// bindings definitions.
template <size_t Peer0, size_t Peer1>
class process_attribute<drake::pydrake::internal::ref_cycle<Peer0, Peer1>>
    : public process_attribute_default<
          drake::pydrake::internal::ref_cycle<Peer0, Peer1>> {
 public:
  // NOLINTNEXTLINE(runtime/references)
  static void precall(function_call& call) {
    // Only generate code if this invocation doesn't need the return value.
    if constexpr (!needs_return_value()) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, handle());
    }
  }

  // NOLINTNEXTLINE(runtime/references)
  static void postcall(function_call& call, handle ret) {
    // Only generate code if this invocation *does* need the return value.
    if constexpr (needs_return_value()) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, ret);
    }
  }

 private:
  // Returns true if either template parameter denotes the return value.
  static constexpr bool needs_return_value() {
    return Peer0 == 0 || Peer1 == 0;
  }
};

}  // namespace detail
}  // namespace pybind11
