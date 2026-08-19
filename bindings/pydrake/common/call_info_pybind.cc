#include "drake/bindings/pydrake/common/call_info_pybind.h"

#include <fmt/format.h>

#include "drake/common/unused.h"

namespace drake {
namespace pydrake {
namespace internal {

const char* FunctionName(const CallInfo& info) {
#ifdef PYDRAKE_USE_PYBIND11
  const char* const function_name = info.call.func.name;
#else   // PYDRAKE_USE_NANOBIND
  // TODO(rpoyner-tri) Figure out if policies can capture the function name for
  // use in error messages.
  unused(info);
  const char* const function_name = "<unknown_function>";
#endif  // PYDRAKE_USE_PYBIND11
  return function_name;
}

py::handle GetArg(const CallInfo& info, py::handle return_value,
    size_t arg_index, const char* policy_name) {
  if (arg_index == 0) {
    return return_value;
  }

#ifdef PYDRAKE_USE_PYBIND11
  if (arg_index == 1 && info.call.init_self) {
    return info.call.init_self;
  }
  if (arg_index <= info.call.args.size()) {
    return info.call.args[arg_index - 1];
  }
  py::pybind11_fail(fmt::format(
      "Could not activate {}: index {} is invalid for function '{}'",
      policy_name, arg_index, FunctionName(info)));
#else   // PYDRAKE_USE_NANOBIND
  if (arg_index <= info.nargs) {
    return info.args[arg_index - 1];
  }
  throw std::runtime_error(fmt::format(
      "Could not activate {}: index {} is invalid for function '{}'",
      policy_name, arg_index, FunctionName(info)));
#endif  // PYDRAKE_USE_PYBIND11
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
