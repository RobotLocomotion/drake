#include "drake/bindings/pydrake/common/ensure_callback_lifetime_pybind.h"

#include <cstdint>

#include <fmt/format.h>

namespace drake {
namespace pydrake {

using py::handle;

namespace internal {

namespace {

// This helper function works around a peculiarity of pybind11 wrapping of
// python bound methods for calls from C++. It is possible to end up with
// distinct call wrappers of the same method, with independent memory, but that
// *compare* as equal. This quirk defeats the reference bookkeeping of
// common/ref_cycle_pybind; attempts to add distinct but "equal" wrappers fail,
// and a reference is lost. To compensate, this function wraps a potentially
// quirky callable in a tuple with its own address, so that apparently "equal"
// callables can be inserted as distinct set members with the
// common/ref_cycle_pybind bookkeeping scheme.
//
// Using the address as the tiebreaker is sound because the bookkeeping set
// holds a strong reference to each callable that it contains, and never
// removes any. Hence an address cannot be reused while its tuple is in the
// set, so distinct callables always yield distinct tuples. Adding the very same
// callable twice yields equal tuples, which is correct because that callable
// is already alive and already the one that C++ refers to.
py::object UniquelyWrapCallback(py::object callback) {
  const auto address = reinterpret_cast<uintptr_t>(callback.ptr());
  return py::make_tuple(callback, address);
}

}  // namespace

void ensure_callback_lifetime_impl(size_t Nurse, size_t CallbackPatient,
    const CallInfo& info, py::handle ret) {
  const char* const policy = "ensure_callback_lifetime";
  handle nurse = info.GetArg(ret, Nurse, policy);
  py::object callback_patient =
      py::borrow<py::object>(info.GetArg(ret, CallbackPatient, policy));
  internal::make_arbitrary_ref_link(
      nurse, UniquelyWrapCallback(callback_patient), info.FunctionName());
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
