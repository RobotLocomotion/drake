#pragma once

/// @file
/// Provides access to Python deprecation utilities from C++.
/// For example usages, please see `deprecation_example/cc_module_py.cc`.

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

// N.B. We cannot use `py::str date = py::none()` because the pybind Python C++
// API converts `None` to a string.
// See: https://github.com/pybind/pybind11/issues/2361

/// Deprecates an attribute `name` of a class `cls`.
/// This *only* works with class attributes (unbound members or methods) as it
/// is implemented with a Python property descriptor.
inline void DeprecateAttribute(py::object cls, py::str name, py::str message,
    std::optional<std::string> date = {}) {
  py::object deprecated =
      py::module::import("pydrake.common.deprecation").attr("deprecated");
  py::object original = cls.attr(name);
  cls.attr(name) = deprecated(message, py::arg("date") = date)(original);
}

/// Raises a deprecation warning.
///
/// @note If you are deprecating a class's member or method, please use
/// `DeprecateAttribute` so that the warning is issued immediately when
/// accessed, not only when it is called.
inline void WarnDeprecated(
    py::str message, std::optional<std::string> date = {}) {
  py::object warn_deprecated =
      py::module::import("pydrake.common.deprecation").attr("_warn_deprecated");
  warn_deprecated(message, py::arg("date") = date);
}

namespace internal {

template <typename Func, typename Return, typename... Args>
auto WrapDeprecatedImpl(py::str message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message](Args... args) {
    WarnDeprecated(message);
    info.func(std::forward<Args>(args)...);
  };
}

// N.B. `decltype(auto)` is used in both places to (easily) achieve perfect
// forwarding of the return type.
template <typename Func, typename Return, typename... Args>
decltype(auto) WrapDeprecatedImpl(py::str message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<!std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message](Args... args) -> decltype(auto) {
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

/// The deprecated flavor of ParamInit<>.
template <typename Class>
auto DeprecatedParamInit(py::str message) {
  return py::init(WrapDeprecated(message, [](py::kwargs kwargs) {
    // N.B. We use `Class` here because `pybind11` strongly requires that we
    // return the instance itself, not just `py::object`.
    // TODO(eric.cousineau): This may hurt `keep_alive` behavior, as this
    // reference may evaporate by the time the true holding pybind11 record is
    // constructed. Would be alleviated using old-style pybind11 init :(
    Class obj{};
    py::object py_obj = py::cast(&obj, py_rvp::reference);
    py::module::import("pydrake").attr("_setattr_kwargs")(py_obj, kwargs);
    return obj;
  }));
}

}  // namespace pydrake
}  // namespace drake
