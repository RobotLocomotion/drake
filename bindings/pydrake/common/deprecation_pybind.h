#pragma once

/// @file
/// Provides access to Python deprecation utilities from C++.
/// For example usages, please see `deprecation_example/cc_module_py.cc`.

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/bindings/pydrake/common/wrap_function.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_export.h"

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
      py::module_::import_("pydrake.common.deprecation").attr("deprecated");
  py::object original = cls.attr(name);
  cls.attr(name) = deprecated(message, py::arg("date") = date)(original);
}

/// Raises a deprecation warning.
///
/// @note If you are deprecating a class's member or method, please use
/// `DeprecateAttribute` so that the warning is issued immediately when
/// accessed, not only when it is called.
inline void WarnDeprecated(
    const std::string& message, std::optional<std::string> date = {}) {
  py::gil_scoped_acquire guard;
  py::object warn_deprecated =
      py::module_::import_("pydrake.common.deprecation")
          .attr("_warn_deprecated");
  warn_deprecated(message, py::arg("date") = date);
}

namespace internal {

template <typename Func, typename Return, typename... Args>
auto WrapDeprecatedImpl(std::string message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message = std::move(message)](Args... args) {
    WarnDeprecated(message);
    info.func(std::forward<Args>(args)...);
  };
}

// N.B. `decltype(auto)` is used in both places to (easily) achieve perfect
// forwarding of the return type.
template <typename Func, typename Return, typename... Args>
decltype(auto) WrapDeprecatedImpl(std::string message,
    function_info<Func, Return, Args...>&& info,
    std::enable_if_t<!std::is_same_v<Return, void>, void*> = {}) {
  return [info = std::move(info), message = std::move(message)](
             Args... args) -> decltype(auto) {
    WarnDeprecated(message);
    return info.func(std::forward<Args>(args)...);
  };
}

}  // namespace internal

/// Wraps any callable (function pointer, method pointer, lambda, etc.) to emit
/// a deprecation message.
template <typename Func>
auto WrapDeprecated(std::string message, Func&& func) {
  return internal::WrapDeprecatedImpl(std::move(message),
      internal::infer_function_info(std::forward<Func>(func)));
}

#ifdef PYDRAKE_USE_NANOBIND
namespace internal {

/// Deprecated wrapping of `py::init<>`.
template <typename CppClass, typename... Args>
struct DRAKE_NO_EXPORT PyInitDeprecatedCtorImpl
    : py::def_visitor<PyInitDeprecatedCtorImpl<CppClass, Args...>> {
  explicit PyInitDeprecatedCtorImpl(std::string message)
      : message_(std::move(message)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__",
        [message = std::move(message_)](CppClass* self, Args... args) {
          WarnDeprecated(message);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
          new (self) CppClass(std::forward<Args>(args)...);
#pragma GCC diagnostic pop
        });
  }
  std::string message_;
};

/// Deprecated wrapping of `py::init(placement-new-callable)`.
template <typename Func>
struct PyInitDeprecatedCustomImpl
    : py::def_visitor<PyInitDeprecatedCustomImpl<Func>> {
  PyInitDeprecatedCustomImpl(std::string message, Func&& func)
      : message_(std::move(message)), func_(std::forward<Func>(func)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__",
        WrapDeprecated(std::move(message_), std::forward<Func>(func_)));
  }
  std::string message_;
  Func&& func_;
};

}  // namespace internal
#endif  // PYDRAKE_USE_NANOBIND

/// Deprecated wrapping of `py::init<>`.
/// @note Only for `unique_ptr` holders. If using `shared_ptr`, talk to Eric.
template <typename Class, typename... Args>
auto py_init_deprecated(std::string message) {
  // N.B. For simplicity, require that Class be passed up front, rather than
  // trying to figure out how to pipe code into / mock `py::detail::initimpl`
  // classes.
#ifdef PYDRAKE_USE_PYBIND11
  return py::init([message = std::move(message)](Args... args) {
    WarnDeprecated(message);
    return std::make_unique<Class>(std::forward<Args>(args)...);
  });
#else  // PYDRAKE_USE_NANOBIND
  return internal::PyInitDeprecatedCtorImpl<Class, Args...>(message);
#endif
}

/// Deprecated wrapping of `py::init(factory)`.
template <typename Func>
auto py_init_deprecated(std::string message, Func&& func) {
#ifdef PYDRAKE_USE_PYBIND11
  return py::init(WrapDeprecated(std::move(message), std::forward<Func>(func)));
#else  // PYDRAKE_USE_NANOBIND
  return internal::PyInitDeprecatedCustomImpl<Func>(message, std::move(func));
#endif
}

/// The deprecated flavor of ParamInit<>.
#ifdef PYDRAKE_USE_PYBIND11
template <typename Class>
auto DeprecatedParamInit(std::string message) {
  return py::init(WrapDeprecated(std::move(message), [](py::kwargs kwargs) {
    // N.B. We use `Class` here because `pybind11` strongly requires that we
    // return the instance itself, not just `py::object`.
    // TODO(eric.cousineau): This may hurt `keep_alive` behavior, as this
    // reference may evaporate by the time the true holding pybind11 record is
    // constructed. Would be alleviated using old-style pybind11 init :(
    Class obj{};
    py::object py_obj = py::cast(&obj, py_rvp::reference);
    py::module_::import_("pydrake").attr("_setattr_kwargs")(py_obj, kwargs);
    return obj;
  }));
}
#else   // PYDRAKE_USE_NANOBIND
template <typename CppClass>
struct DRAKE_NO_EXPORT DeprecatedParamInit
    : py::def_visitor<DeprecatedParamInit<CppClass>> {
  explicit DeprecatedParamInit(std::string message)
      : message_(std::move(message)) {}
  template <typename Class, typename... Extra>
  void execute(Class& cl, const Extra&...) {
    cl.def("__init__", WrapDeprecated(std::move(message_),
                           [](CppClass* self, py::kwargs kwargs) {
                             new (self) Class();
                             py::object py_obj =
                                 py::cast(self, py_rvp::reference);
                             py::module_::import_("pydrake").attr(
                                 "_setattr_kwargs")(py_obj, kwargs);
                           }));
  }
  std::string message_;
};
#endif  // PYDRAKE_USE_PYBIND11

}  // namespace pydrake
}  // namespace drake
