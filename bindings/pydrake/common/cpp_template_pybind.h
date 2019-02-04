#pragma once

#include <string>
#include <utility>

#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

// `GetPyTypes` is implemented specifically for `cpp_template`; to simplify
// dependencies, this is included transitively.
#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

// C++ interface for `pydrake.common.cpp_template.get_or_init`.
// Please see that function for common parameters.
// @param template_cls_name Name of the template class in `cpp_template`,
// resolves to class to be passed as `template_cls`.
inline py::object GetOrInitTemplate(  // BR
    py::handle scope, const std::string& name,
    const std::string& template_cls_name,  // BR
    py::tuple args = py::tuple(), py::dict kwargs = py::dict()) {
  const char module_name[] = "pydrake.common.cpp_template";
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
  return py::cast<std::string>(py_template.attr("_instantiation_name")(param));
}

}  // namespace internal

/// Provides a temporary, unique name for a class instantiation that
/// will be passed to `AddTemplateClass`.
template <typename T>
std::string TemporaryClassName(const std::string& name = "TemporaryName") {
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
inline py::object AddTemplateClass(  // BR
    py::handle scope, const std::string& name, py::handle py_class,
    py::tuple param) {
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateClass");
  internal::AddInstantiation(py_template, py_class, param);
  return py_template;
}

/// Provides a convenience wrapper for defining a template class instantiation
/// and a default instantiation (if not already defined).
/// The default instantiation is named `default_name`, while the template is
/// named `default_name + template_suffix`.
/// @return pybind11 class
template <typename Class, typename... Options>
py::class_<Class, Options...> DefineTemplateClassWithDefault(  // BR
    py::handle scope, const std::string& default_name, py::tuple param,
    const char* doc_string = "", const std::string& template_suffix = "_") {
  const std::string template_name = default_name + template_suffix;
  // Define class with temporary name.
  py::class_<Class, Options...> py_class(
      scope, TemporaryClassName<Class>().c_str(), doc_string);
  // Register instantiation.
  AddTemplateClass(scope, template_name, py_class, param);
  // Declare default instantiation if it does not already exist.
  if (!py::hasattr(scope, default_name.c_str())) {
    scope.attr(default_name.c_str()) = py_class;
  }
  return py_class;
}

/// Declares a template function.
/// @param scope Parent scope of the template.
/// @param name Name of the template.
/// @param func Function to be added.
/// @param param Parameters for the instantiation.
/// @param extra... Additional arguments to pass to `py::cpp_function`.
template <typename Func, typename... Extra>
py::object AddTemplateFunction(py::handle scope, const std::string& name,
    Func&& func, py::tuple param, Extra&&... extra) {
  // TODO(eric.cousineau): Use `py::sibling` if overloads are needed.
  py::object py_template =
      internal::GetOrInitTemplate(scope, name, "TemplateFunction");
  py::object py_func = py::cpp_function(  // BR
      std::forward<Func>(func),
      py::name(internal::GetInstantiationName(py_template, param).c_str()),
      std::forward<Extra>(extra)...);
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

/// Declares a template method.
/// @param scope Parent scope of the template. This should be a class.
/// @param name Name of the template.
/// @param method Method to be added.
/// @param param Parameters for the instantiation.
/// @param extra... Additional arguments to pass to `py::cpp_function`.
template <typename Method, typename... Extra>
py::object AddTemplateMethod(  // BR
    py::handle scope, const std::string& name, Method&& method, py::tuple param,
    Extra&&... extra) {
  py::object py_template = internal::GetOrInitTemplate(
      scope, name, "TemplateMethod", py::make_tuple(scope));
  py::object py_func = py::cpp_function(  // BR
      std::forward<Method>(method),
      py::name(internal::GetInstantiationName(py_template, param).c_str()),
      py::is_method(scope), std::forward<Extra>(extra)...);
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

}  //  namespace pydrake
}  //  namespace drake
