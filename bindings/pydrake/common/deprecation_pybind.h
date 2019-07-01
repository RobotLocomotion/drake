#pragma once

/// @file
/// Provides access to Python deprecation utilities from C++.
/// For example usages, please see `deprecation_example/cc_module_py.cc`.

#include <memory>
#include <utility>

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/// Deprecates an attribute `name` of a class `cls`.
/// This *only* works with class attributes (unbound members or methods) as it
/// is implemented with a Python property descriptor.
inline void DeprecateAttribute(py::object cls, py::str name, py::str message) {
  py::object deprecated =
      py::module::import("pydrake.common.deprecation").attr("deprecated");
  py::object original = cls.attr(name);
  cls.attr(name) = deprecated(message)(original);
}

/// Raises a deprecation warning.
///
/// @note If you are deprecating a class's member or method, please use
/// `DeprecateAttribute` so that the warning is issued immediately when
/// accessed, not only when it is called.
inline void WarnDeprecated(py::str message) {
  py::object warn_deprecated =
      py::module::import("pydrake.common.deprecation").attr("_warn_deprecated");
  warn_deprecated(message);
}

namespace internal {

template <typename Func, typename Return, typename... Args>
auto WrapDeprecatedImpl(py::str message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<std::is_same<Return, void>::value, void*> = {}) {
  return [info = std::move(info), message](Args... args) {
    WarnDeprecated(message);
    info.func(std::forward<Args>(args)...);
  };
}

template <typename Func, typename Return, typename... Args>
auto WrapDeprecatedImpl(py::str message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<!std::is_same<Return, void>::value, void*> = {}) {
  return [info = std::move(info), message](Args... args) -> Return {
    WarnDeprecated(message);
    return info.func(std::forward<Args>(args)...);
  };
}

}  // namespace internal

/// Wraps any callable (function pointer, method pointer, lambda, etc.) to emit
/// a deprecation message.
template <typename Func>
auto WrapDeprecated(py::str message, Func&& func) {
  return internal::WrapDeprecatedImpl(
      message, internal::infer_function_info(std::forward<Func>(func)));
}

/// Deprecated wrapping of `py::init<>`.
/// @note Only for `unique_ptr` holders. If using `shared_ptr`, talk to Eric.
template <typename Class, typename... Args>
auto py_init_deprecated(py::str message) {
  // N.B. For simplicity, require that Class be passed up front, rather than
  // trying to figure out how to pipe code into / mock `py::detail::initimpl`
  // classes.
  return py::init([message](Args... args) {
    WarnDeprecated(message);
    return std::make_unique<Class>(std::forward<Args>(args)...);
  });
}

/// Deprecated wrapping of `py::init(factory)`.
template <typename Func>
auto py_init_deprecated(py::str message, Func&& func) {
  return py::init(WrapDeprecated(message, std::forward<Func>(func)));
}

}  // namespace pydrake
}  // namespace drake
