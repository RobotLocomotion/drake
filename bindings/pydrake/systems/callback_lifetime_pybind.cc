#include "drake/bindings/pydrake/systems/callback_lifetime_pybind.h"

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
// quirky callable in a tuple with a unique integer, so that apparently "equal"
// callables can be inserted as distinct set members with the
// common/ref_cycle_pybind bookkeeping scheme.
py::object UniquelyWrapCallback(py::object callback) {
  static std::atomic<uint64_t> uniquifier{0};
  return py::make_tuple(callback, uniquifier.fetch_add(1));
}

}  // namespace

void ensure_callback_lifetime_impl(size_t Nurse, size_t CallbackPatient,
    const CallInfo& info, py::handle ret) {
  const char* const policy = "ensure_callback_lifetime";
  handle nurse = GetArg(info, ret, Nurse, policy);
  py::object callback_patient =
      py::borrow<py::object>(GetArg(info, ret, CallbackPatient, policy));
  internal::make_arbitrary_ref_link(
      nurse, UniquelyWrapCallback(callback_patient), FunctionName(info));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
