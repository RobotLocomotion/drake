#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Encapsulates function call information available from the binder library to
call policies. */
class __attribute__((visibility("hidden"))) CallInfo {
 public:
#ifdef PYDRAKE_USE_PYBIND11
  CallInfo(const py::detail::function_call& call) : call_(call) {}
#else   // PYDRAKE_USE_NANOBIND
  CallInfo(PyObject** args, size_t nargs) : args_(args), nargs_(nargs) {}
#endif  // PYDRAKE_USE_PYBIND11

  /* Returns the the name of the function being bound, if available, or a
  place-holder name otherwise. */
  const char* FunctionName() const;

  /* Returns the argument handle selected by the given index.

  @throws if the index is invalid.

  The argument index starts at 1; for methods, `self` is at index 1. Index 0
  denotes the return value. */
  py::handle GetArg(
      py::handle return_value, size_t arg_index, const char* policy_name) const;

 private:
#ifdef PYDRAKE_USE_PYBIND11
  const py::detail::function_call& call_;
#else   // PYDRAKE_USE_NANOBIND
  PyObject** args_{};
  size_t nargs_{};
#endif  // PYDRAKE_USE_PYBIND11
};

/* Returns true if any argument index template parameter denotes the return
value. */
template <size_t... Indices>
consteval bool needs_return_value() {
  bool result{0};
  ((result |= (Indices == 0)), ...);
  return result;
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
