#pragma once

#include <string>
#include <utility>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
// `GetPyTypes` is implemented specifically for `cpp_template`; to simplify
// dependencies, this is included transitively.
#include "drake/bindings/pydrake/util/cpp_param_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

// C++ interface for `pydrake.util.cpp_template.get_or_init`.
// Please see that function for common parameters.
// @param template_cls_name Name of the template class in `cpp_template`,
// resolves to class to be passed as `template_cls`.
inline py::object GetOrInitTemplate(
    py::handle scope, const std::string& name,
    const std::string& template_cls_name,
    py::tuple args = py::tuple(), py::dict kwargs = py::dict()) {
  const char module_name[] = "pydrake.util.cpp_template";
  py::handle m = py::module::import(module_name);
  return m.attr("get_or_init")(
      scope, name, m.attr(template_cls_name.c_str()), *args, **kwargs);
}

// Adds instantiation to a Python template.
inline void AddInstantiation(
    py::handle py_template, py::handle obj, py::tuple param) {
  py_template.attr("add_instantiation")(param, obj);
}

// Gets name for a given instantiation.
inline std::string GetInstantiationName(
    py::handle py_template, py::tuple param) {
  return py::cast<std::string>(
    py_template.attr("_instantiation_name")(param));
}

}  // namespace internal


/// Provides a temporary, unique name for a class instantiation that
/// will be passed to `AddTemplateClass`.
template <typename T>
std::string TemporaryClassName(
    const std::string& name = "TemporaryName") {
  return "_" + name + "_" + typeid(T).name();
}

/// Adds a template class instantiation.
/// @param scope Parent scope of the template.
/// @param name Name of the template.
/// @param py_class Class instantiation to be added.
/// @note The class name should be *unique*. If you would like automatic unique
/// names, consider constructing the class binding as
/// `py::class_<Class, ...>(m, TemporaryClassName<Class>().c_str())`.
/// @param param Parameters for the instantiation.
inline py::object AddTemplateClass(
    py::handle scope, const std::string& name,
    py::handle py_class, py::tuple param) {
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateClass");
  internal::AddInstantiation(py_template, py_class, param);
  return py_template;
}

/// Declares a template function.
/// @param scope Parent scope of the template.
/// @param name Name of the template.
/// @param func Function to be added.
/// @param param Parameters for the instantiation.
template <typename Func>
py::object AddTemplateFunction(
    py::handle scope, const std::string& name, Func&& func,
    py::tuple param) {
  // TODO(eric.cousineau): Use `py::sibling` if overloads are needed.
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateFunction");
  py::object py_func = py::cpp_function(
        std::forward<Func>(func),
        py::name(internal::GetInstantiationName(py_template, param).c_str()));
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

/// Declares a template method.
/// @param scope Parent scope of the template. This should be a class.
/// @param name Name of the template.
/// @param method Method to be added.
/// @param param Parameters for the instantiation.
template <typename Method>
py::object AddTemplateMethod(
    py::handle scope, const std::string& name, Method&& method,
    py::tuple param) {
  py::object py_template =
      internal::GetOrInitTemplate(
          scope, name, "TemplateMethod", py::make_tuple(scope));
  py::object py_func = py::cpp_function(
      std::forward<Method>(method),
      py::name(internal::GetInstantiationName(py_template, param).c_str()),
      py::is_method(scope));
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

}  //  namespace pydrake
}  //  namespace drake
