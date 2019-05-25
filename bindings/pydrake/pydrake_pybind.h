#pragma once

#include "pybind11/pybind11.h"

// N.B. Avoid including other headers, such as `pybind11/eigen.sh` or
// `pybind11/functional.sh`, such that modules can opt-in to (and pay the cost
// for) these binding capabilities.

namespace drake {
namespace pydrake {

// Note: Doxygen apparently doesn't process comments for namespace aliases. If
// you put Doxygen comments here they will apply instead to py_reference. See
// the "Convenience aliases" section above for documentation.
namespace py = pybind11;

/// Used when returning `T& or `const T&`, as pybind's default behavior is to
/// copy lvalue references.
const auto py_reference = py::return_value_policy::reference;

/// Used when returning references to objects that are internally owned by
/// `self`. Implies both `py_reference` and `py::keep_alive<0, 1>`, which
/// implies "Keep alive, reference: `return` keeps` self` alive".
const auto py_reference_internal = py::return_value_policy::reference_internal;

/// Use this when you must do manual casting - e.g. lists or tuples of nurses,
/// where the container may get discarded but the items kept. Prefer this over
/// `py::cast(obj, reference_internal, parent)` (pending full resolution of
/// #11046).
inline py::object py_keep_alive(py::object nurse, py::object patient) {
  py::detail::keep_alive_impl(nurse, patient);
  return nurse;
}

// Implementation for `overload_cast_explicit`. We must use this structure so
// that we can constrain what is inferred. Otherwise, the ambiguity confuses
// the compiler.
template <typename Return, typename... Args>
struct overload_cast_impl {
  auto operator()(Return (*func)(Args...)) const { return func; }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...)) const {
    return method;
  }

  template <typename Class>
  auto operator()(Return (Class::*method)(Args...) const) const {
    return method;
  }
};

/// Provides option to provide explicit signature when
/// `py::overload_cast<Args...>` fails to infer the Return argument.
template <typename Return, typename... Args>
constexpr auto overload_cast_explicit = overload_cast_impl<Return, Args...>{};

/// Binds Pythonic `__copy__` and `__deepcopy__` using class's copy
/// constructor.
/// @note Do not use this if the class's copy constructor does not imply a deep
/// copy.
template <typename PyClass>
void DefCopyAndDeepCopy(PyClass* ppy_class) {
  using Class = typename PyClass::type;
  PyClass& py_class = *ppy_class;
  py_class.def("__copy__", [](const Class* self) { return Class{*self}; })
      .def("__deepcopy__",
          [](const Class* self, py::dict /* memo */) { return Class{*self}; });
}

/// Binds Pythonic `__copy__` and `__deepcopy__` for a class, as well as
/// `Clone` method, using class's `Clone` method rather than the copy
/// constructor.
template <typename PyClass>
void DefClone(PyClass* ppy_class) {
  using Class = typename PyClass::type;
  PyClass& py_class = *ppy_class;
  py_class  // BR
      .def("Clone", &Class::Clone)
      .def("__copy__", &Class::Clone)
      .def("__deepcopy__",
          [](const Class* self, py::dict /* memo */) { return self->Clone(); });
}

/// Executes Python code to introduce additional symbols for a given module.
/// For a module with local name `{name}`, the code executed will be
/// `_{name}_extra.py`. See #9599 for relevant background.
inline void ExecuteExtraPythonCode(py::module m) {
  py::module::import("pydrake").attr("_execute_extra_python_code")(m);
}

#if PY_MAJOR_VERSION >= 3
// The following works around pybind11 modules getting reconstructed /
// reimported in Python3. See pybind/pybind11#1559 for more details.
// Use this ONLY when necessary (e.g. when using a utility method which imports
// the module, within the module itself).
#define PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(variable)                 \
  {                                                                       \
    static py::handle variable##_original;                                \
    if (variable##_original) {                                            \
      variable = py::reinterpret_borrow<py::module>(variable##_original); \
      return;                                                             \
    } else {                                                              \
      variable##_original = variable;                                     \
    }                                                                     \
  }
#else  // PY_MAJOR_VERSION >= 3
// N.B. Still use the variable to ensure it's valid code.
#define PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(variable) \
  { (void)variable; }
#endif  // PY_MAJOR_VERSION >= 3

}  // namespace pydrake
}  // namespace drake
