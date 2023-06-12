#pragma once

#include <string>
#include <utility>

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
inline void AddInstantiation(py::handle py_template, py::handle obj,
    py::tuple param, bool skip_rename = false) {
  py_template.attr("add_instantiation")(param, obj, skip_rename);
}

// Gets name for a given instantiation.
inline std::string GetInstantiationName(
    py::handle py_template, py::tuple param, bool mangle = false) {
  return py::cast<std::string>(py_template.attr("_instantiation_name")(
      param, py::arg("mangle") = mangle));
}

// C++ wrapper around pydrake.common.pretty_class_name.
inline py::object PrettyClassName(py::handle cls, bool use_qualname = false) {
  py::handle py_func =
      py::module::import("pydrake.common").attr("pretty_class_name");
  return py_func(cls, py::arg("use_qualname") = use_qualname);
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
/// @param template_name Name of the template.
/// @param py_class Class instantiation to be added.
/// @note The class name should be *unique*. If you would like automatic unique
/// names, consider constructing the class binding as
/// `py::class_<Class, ...>(m, TemporaryClassName<Class>().c_str())`.
/// @param param Parameters for the instantiation.
inline py::object AddTemplateClass(  // BR
    py::handle scope, const std::string& template_name, py::handle py_class,
    py::tuple param, bool skip_rename = false) {
  py::object py_template =
      internal::GetOrInitTemplate(scope, template_name, "TemplateClass");
  internal::AddInstantiation(py_template, py_class, param, skip_rename);
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
  // The default instantiation is immediately assigned its correct class name.
  // Other instantiations use a temporary name here that will be overwritten
  // by the AddTemplateClass function during registration.
  const bool is_default = !py::hasattr(scope, default_name.c_str());
  const std::string class_name =
      is_default ? default_name : TemporaryClassName<Class>();
  const std::string template_name = default_name + template_suffix;
  // Define the class.
  std::string doc;
  if (is_default) {
    doc = fmt::format(
        "{}\n\nNote:\n\n"
        "    This class is templated; see :class:`{}`\n"
        "    for the list of instantiations.",
        doc_string, template_name);
  } else {
    doc = doc_string;
  }
  py::class_<Class, Options...> py_class(
      scope, class_name.c_str(), doc.c_str());
  // Register it as a template instantiation.
  const bool skip_rename = is_default;
  AddTemplateClass(scope, template_name, py_class, param, skip_rename);
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
  const bool mangle = true;
  const std::string instantiation_name =
      internal::GetInstantiationName(py_template, param, mangle);
  py::object py_func = py::cpp_function(std::forward<Func>(func),
      py::name(instantiation_name.c_str()), std::forward<Extra>(extra)...);
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
  const bool mangle = true;
  const std::string instantiation_name =
      internal::GetInstantiationName(py_template, param, mangle);
  py::object py_func = py::cpp_function(std::forward<Method>(method),
      py::name(instantiation_name.c_str()), py::is_method(scope),
      std::forward<Extra>(extra)...);
  internal::AddInstantiation(py_template, py_func, param);
  return py_template;
}

}  //  namespace pydrake
}  //  namespace drake
