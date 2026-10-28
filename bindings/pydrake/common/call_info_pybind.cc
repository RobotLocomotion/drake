#include "drake/bindings/pydrake/common/call_info_pybind.h"

#include <fmt/format.h>

namespace drake {
namespace pydrake {
namespace internal {

const char* CallInfo::FunctionName() const {
#ifdef PYDRAKE_USE_PYBIND11
  const char* const function_name = call_.func.name;
#else   // PYDRAKE_USE_NANOBIND
  // TODO(rpoyner-tri) Figure out if policies can capture the function name for
  // use in error messages.
  const char* const function_name = "<unknown_function>";
#endif  // PYDRAKE_USE_PYBIND11
  return function_name;
}

py::handle CallInfo::GetArg(
    py::handle return_value, size_t arg_index, const char* policy_name) const {
  if (arg_index == 0) {
    return return_value;
  }

#ifdef PYDRAKE_USE_PYBIND11
  if (arg_index == 1 && call_.init_self) {
    return call_.init_self;
  }
  if (arg_index <= call_.args.size()) {
    return call_.args[arg_index - 1];
  }
  py::pybind11_fail(fmt::format(
      "Could not activate {}: index {} is invalid for function '{}'",
      policy_name, arg_index, FunctionName()));
#else   // PYDRAKE_USE_NANOBIND
  if (arg_index <= nargs_) {
    return args_[arg_index - 1];
  }
  throw std::runtime_error(fmt::format(
      "Could not activate {}: index {} is invalid for function '{}'",
      policy_name, arg_index, FunctionName()));
#endif  // PYDRAKE_USE_PYBIND11
}

// Tests for needs_return_value.
static_assert(needs_return_value() == false);
static_assert(needs_return_value<1>() == false);
static_assert(needs_return_value<0>() == true);
static_assert(needs_return_value<1, 2>() == false);
static_assert(needs_return_value<0, 3>() == true);
static_assert(needs_return_value<7, 0>() == true);
static_assert(needs_return_value<1, 4, 2, 8, 5, 7>() == false);
static_assert(needs_return_value<9, 5, 6, 0, 1, 3>() == true);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
