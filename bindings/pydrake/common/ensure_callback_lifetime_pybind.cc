#include "drake/bindings/pydrake/common/ensure_callback_lifetime_pybind.h"

namespace drake {
namespace pydrake {

using py::handle;

namespace internal {

void ensure_callback_lifetime_impl(size_t Nurse, size_t CallbackPatient,
    const CallInfo& info, py::handle ret) {
  const char* const policy = "ensure_callback_lifetime";
  handle nurse = info.GetArg(ret, Nurse, policy);
  handle callback_patient = info.GetArg(ret, CallbackPatient, policy);
  internal::make_arbitrary_ref_link(
      nurse, callback_patient, info.FunctionName());
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
