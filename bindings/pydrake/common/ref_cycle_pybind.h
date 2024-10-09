#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* pydrake::internal::ref_cycle creates a reference count cycle that Python's
  cyclic garbage collection can find and collect, once the cycle's objects are
  no longer reachable. Both peer objects must by pybind11 classes that were
  defined with the dynamic_attr() annotation.

  Wherever pybind11::keep_alive<M, N>() can be used, ref_cycle<M, N>() can be
  used similarly.
 */
template <size_t Peer0, size_t Peer1>
struct ref_cycle {};

/* This function is used in the template below to select peers by call/return
 index. */
void ref_cycle_impl(size_t Peer0, size_t Peer1,
    const pybind11::detail::function_call& call, pybind11::handle ret);

/* This function creates the cycle, given peers as pybind11::handles. */
void do_ref_cycle_impl(pybind11::handle p0, pybind11::handle p1);

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
    if constexpr (!needs_result()) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, handle());
    }
  }

  // NOLINTNEXTLINE(runtime/references)
  static void postcall(function_call& call, handle ret) {
    if constexpr (needs_result()) {
      drake::pydrake::internal::ref_cycle_impl(Peer0, Peer1, call, ret);
    }
  }

 private:
  static constexpr bool needs_result() { return Peer0 == 0 || Peer1 == 0; }
};

}  // namespace detail
}  // namespace pybind11
