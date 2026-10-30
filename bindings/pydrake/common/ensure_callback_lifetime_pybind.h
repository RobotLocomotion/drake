#pragma once

#include <string>

#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* internal::ensure_callback_lifetime is a custom call policy for pydrake.

For an overview of other call policies, See
https://pybind11.readthedocs.io/en/stable/advanced/functions.html#additional-call-policies
https://nanobind.readthedocs.io/en/latest/functions.html#lifetime-annotations

`ensure_callback_lifetime` creates a one-way reference from an arbitrary object
to a Python callable object that Python's garbage collection can track. Even
though the immediately constructed reference is one-way, compatibility with
garbage collection is necessary to handle callables that capture Python
variables and thereby create reference cycles.

This policy uses the same underlying technique and implementation as
pydrake::internal::ref_cycle. Python callable objects need special lifetime
treatment because (1) they cannot in general participate directly in a
pydrake::internal::ref_cycle, and (2) even tracking their identities requires
special handling.

For the special case of binding custom constructors, the
ensure-callback-lifetime treatment can only be applied as a call policy. This
is because casting `self` to python inside a custom constructor lambda "works"
in an unexpected way: it creates a python "temporary" that is immediately
discarded, and that is unrelated to the completed instance ultimately delivered
to the calling python code.

Note the consequences for object lifetimes:

* Nurse keeps CallbackPatient alive.
* If the reference forms part of a cycle via captures:
  * the cycle can be garbage collected once all member objects are unreachable.

@tparam Nurse an argument index to a `ref_cycle` compatible object
@tparam CallbackPatient an argument index to a callable

The argument index starts at 1; for methods, `self` is at index 1. Index 0
denotes the return value. */
template <size_t Nurse, size_t CallbackPatient>
struct ensure_callback_lifetime {};

/* This function is used in the template below to select participants by
call/return index. */
void ensure_callback_lifetime_impl(size_t nurse, size_t callback_patient,
    const CallInfo& info, py::handle ret);

// For nanobind annotations, we must provide ADL hooks for func_extra_info.
#ifdef PYDRAKE_USE_NANOBIND
template <size_t NArgs, size_t Nurse, size_t CallbackPatient>
NB_INLINE void process_precall(PyObject** args,
    std::integral_constant<size_t, NArgs>, nanobind::detail::cleanup_list*,
    ensure_callback_lifetime<Nurse, CallbackPatient>*) {
  if constexpr (!needs_return_value<Nurse, CallbackPatient>()) {
    ensure_callback_lifetime_impl(
        Nurse, CallbackPatient, {args, NArgs}, nanobind::handle());
  }
}

template <size_t NArgs, size_t Nurse, size_t CallbackPatient>
void process_postcall(PyObject** args, std::integral_constant<size_t, NArgs>,
    PyObject* result, ensure_callback_lifetime<Nurse, CallbackPatient>*) {
  if constexpr (needs_return_value<Nurse, CallbackPatient>()) {
    // result_guard avoids leaking a reference to the return object if postcall
    // throws an exception.
    py::object result_guard = py::steal(result);
    ensure_callback_lifetime_impl(
        Nurse, CallbackPatient, {args, NArgs}, result);
    result_guard.release();
  }
}

template <typename F, size_t Nurse, size_t CallbackPatient>
NB_INLINE void func_extra_apply(
    F& f, ensure_callback_lifetime<Nurse, CallbackPatient>, size_t&) {
  f.flags |=
      static_cast<uint32_t>(nanobind::detail::func_flags::can_mutate_args);
}
#endif  // PYDRAKE_USE_NANOBIND

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace PYDRAKE_BINDER_NAMESPACE {
namespace detail {

// Provide specializations of the internal templates for call policies; this
// allows writing an annotation that works seamlessly in bindings definitions.
#ifdef PYDRAKE_USE_PYBIND11
template <size_t Nurse, size_t CallbackPatient>
class process_attribute<
    drake::pydrake::internal::ensure_callback_lifetime<Nurse, CallbackPatient>>
    : public process_attribute_default<drake::pydrake::internal::
              ensure_callback_lifetime<Nurse, CallbackPatient>> {
 public:
  // NOLINTNEXTLINE(runtime/references)
  static void precall(function_call& call) {
    if constexpr (!needs_return_value_) {
      drake::pydrake::internal::ensure_callback_lifetime_impl(
          Nurse, CallbackPatient, {call}, handle());
    }
  }

  // NOLINTNEXTLINE(runtime/references)
  static void postcall(function_call& call, handle ret) {
    if constexpr (needs_return_value_) {
      drake::pydrake::internal::ensure_callback_lifetime_impl(
          Nurse, CallbackPatient, {call}, ret);
    }
  }

 private:
  static constexpr bool needs_return_value_ =
      drake::pydrake::internal::needs_return_value<Nurse, CallbackPatient>();
};
#else   // PYDRAKE_USE_NANOBIND
template <size_t Nurse, size_t CallbackPatient, typename... Ts>
struct func_extra_info<
    drake::pydrake::internal::ensure_callback_lifetime<Nurse, CallbackPatient>,
    Ts...> : func_extra_info<Ts...> {
  static constexpr bool pre_post_hooks = true;
};
#endif  // PYDRAKE_USE_PYBIND11

}  // namespace detail
}  // namespace PYDRAKE_BINDER_NAMESPACE
