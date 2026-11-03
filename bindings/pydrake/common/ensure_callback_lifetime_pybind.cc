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
  make_arbitrary_callback_lifetime(
      nurse, callback_patient, info.FunctionName());
}

void make_arbitrary_callback_lifetime(
    py::handle nurse, py::handle callback, const std::string& location_hint) {
  make_arbitrary_ref_link(nurse, callback, location_hint);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
