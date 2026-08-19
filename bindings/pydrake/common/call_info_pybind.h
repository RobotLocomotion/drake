#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Encapsulates function call information available from the binder library to
call policies. */
struct __attribute__((visibility("hidden"))) CallInfo {
#ifdef PYDRAKE_USE_PYBIND11
  const py::detail::function_call& call;
#else   // PYDRAKE_USE_NANOBIND
  PyObject** args{};
  size_t nargs{};
#endif  // PYDRAKE_USE_PYBIND11
};

/* Returns the the name of the function being bound, if available, or a
place-holder name otherwise. */
const char* FunctionName(const CallInfo& info);

/* Returns the argument handle selected by the given index.

@throws if the index is invalid.

The argument index starts at 1; for methods, `self` is at index 1. Index 0
denotes the return value. */
py::handle GetArg(const CallInfo& info, py::handle return_value,
    size_t arg_index, const char* policy_name);

/* Returns true if any argument index template parameter denotes the return
value. */
template <size_t... Indices>
consteval bool needs_return_value() {
  bool result{0};
  ((result |= (Indices == 0)), ...);
  return result;
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
