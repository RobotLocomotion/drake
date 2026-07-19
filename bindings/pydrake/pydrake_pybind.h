#pragma once

#include <memory>
#include <utility>

// Here we include a lot of the pybind11 API, to ensure that all code in pydrake
// sees the same definitions ("One Definition Rule") for template types intended
// for specialization. Any pybind11 headers with `type_caster<>` specializations
// must be included here (e.g., eigen.h, functional.h, numpy.h, stl.h) as well
// as ADL headers (e.g., operators.h). Headers that are unused by pydrake
// (e.g., complex.h) are omitted.
#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/stl/filesystem.h"
#include "pybind11/typing.h"

namespace drake {

/// For more high-level information, see the @ref python_bindings
/// "Python Bindings" technical notes.
///
/// Drake developers should prefer any aliases defined here over their full
/// spellings in `pybind11`.
///
/// `namespace py` is a shorthand alias to `pybind11` for consistency. (This
/// symbol cannot be exposed directly in Doxygen.)
///
/// @note Downstream users should avoid `using namespace drake::pydrake`, as
/// this may create ambiguous aliases (especially for GCC). Instead, consider
/// using your own alias directly to the `pybind11` namespace.
namespace pydrake {

// Note: Doxygen apparently doesn't process comments for namespace aliases. If
// you put Doxygen comments here they will apply instead to py_rvp.
namespace py = pybind11;

/// Shortened alias for py::rv_policy. For more information, see
/// the @ref PydrakeReturnValuePolicy "Return Value Policy" section.
using py_rvp = py::rv_policy;

// This alias helps ease Drake's transition to nanobind.
using py::class_;

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
  using Class = typename PyClass::Type;
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
  // Having abandoned the old RobotLocomotion pybind11 branch
  // with special handling of std::unique_ptr<>, these bindings'
  // return value paths started deleting the C++ object and
  // returning a dead non-null pointer. To avoid that, we
  // instead explicitly unwrap the pointer here and rely on the
  // take_ownership return value policy. The take_ownership
  // policy would be the default policy in this case, but it
  // seems safer and more clear to apply it explicitly.
  using Class = typename PyClass::Type;
  PyClass& py_class = *ppy_class;
  py_class  // BR
      .def(
          "Clone", [](const Class* self) { return self->Clone().release(); },
          py_rvp::take_ownership)
      .def(
          "__copy__", [](const Class* self) { return self->Clone().release(); },
          py_rvp::take_ownership)
      .def(
          "__deepcopy__",
          [](const Class* self, py::dict /* memo */) {
            return self->Clone().release();
          },
          py_rvp::take_ownership);
}

/// Binds `__getstate__` and `__setstate__` for pickling on the given
/// `ppy_class` (which must point to a `class_`).
///
/// The get_state functor should take `(const Class& self)` and return a
/// newly-pickled class `-> Pickled` by value.
///
/// The set_state functor should take `(Class* self, Pickled pickled)` and
/// placement-new construct the object into `self` based on `pickled`, with no
/// return value. (The use of placement new is in anticipation of a nanobind
/// port of this helper function.)
template <typename PyClass, typename GetState, typename SetState>
void DefPickle(PyClass* ppy_class, GetState&& get_state, SetState&& set_state) {
  PyClass& py_class = *ppy_class;

  using Class = typename PyClass::Type;
  using Pickled = std::invoke_result_t<GetState, const Class&>;

  // For pybind11 we must wrap set_state to return the constructed Class by
  // value, instead of using placement new. (Nanobind will use placement new.)
  auto set_state_with_return = [set_state = std::forward<SetState>(set_state)](
                                   Pickled pickled) {
    alignas(Class) std::byte buffer[sizeof(Class)];
    Class* typed_buffer = reinterpret_cast<Class*>(buffer);
    set_state(typed_buffer, std::move(pickled));
    Class result = std::move(*typed_buffer);
    typed_buffer->~Class();
    return result;
  };

  py_class.def(py::pickle(
      std::forward<GetState>(get_state), std::move(set_state_with_return)));
}

/// Returns a constructor for creating an instance of Class and initializing
/// parameters (bound using `def_rw`).
/// This provides an alternative to manually enumerating each
/// parameter as an argument using `py::init<...>` and `py::arg(...)`, and is
/// useful when the C++ class only has a default constructor. Example:
/// @code
/// using Class = ExampleClass;
/// class_<Class>(m, "ExampleClass")  // BR
///     .def(ParamInit<Class>());
/// @endcode
///
/// @tparam Class The C++ class. Must have a default constructor.
template <typename Class>
auto ParamInit() {
  return py::init([](py::kwargs kwargs) {
    // N.B. We use `Class` here because `pybind11` strongly requires that we
    // return the instance itself, not just `py::object`.
    // TODO(eric.cousineau): This may hurt `keep_alive` behavior, as this
    // reference may evaporate by the time the true holding pybind11 record is
    // constructed. Would be alleviated using old-style pybind11 init :(
    Class obj{};
    py::object py_obj = py::cast(&obj, py_rvp::reference);
    py::module_::import_("pydrake").attr("_setattr_kwargs")(py_obj, kwargs);
    return obj;
  });
}

/// Executes Python code to introduce additional symbols for a given module.
/// For a module with local name `{name}` and use_subdir=False, the code
/// executed will be `_{name}_extra.py`; with use_subdir=True, it will be
/// `{name}/_{name}_extra.py`. See #9599 for relevant background.
inline void ExecuteExtraPythonCode(py::module_ m, bool use_subdir = false) {
  py::module_::import_("pydrake").attr("_execute_extra_python_code")(
      m, use_subdir);
}

// The following works around pybind11 modules getting reconstructed /
// reimported in Python3. See pybind/pybind11#1559 for more details.
// Use this ONLY when necessary (e.g. when using a utility method which imports
// the module, within the module itself).
// TODO(eric.cousineau): Unfold cyclic references, and remove the need for this
// macro (see #11868 for rationale).
#define PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(variable)      \
  {                                                            \
    static py::handle variable##_original;                     \
    if (variable##_original) {                                 \
      variable##_original.inc_ref();                           \
      variable = py::borrow<py::module_>(variable##_original); \
      return;                                                  \
    } else {                                                   \
      variable##_original = variable;                          \
    }                                                          \
  }

/// Given a raw pointer, returns a shared_ptr wrapper around it that doesn't own
/// anything -- it's managed object is null, so there is no reference counting.
/// Calling get() on the result will return `raw`.
template <typename T>
std::shared_ptr<T> make_unowned_shared_ptr_from_raw(T* raw) {
  return std::shared_ptr<T>(
      /* managed object = */ std::shared_ptr<void>{},
      /* stored pointer = */ raw);
}

/// Given a Python object, returns a shared_ptr wrapper around it that keeps
/// the Python object alive. If the py_object is None, returns nullptr. You
/// must supply the expected C++ type to cast to as `T`.
template <typename T>
std::shared_ptr<T> make_shared_ptr_from_py_object(py::object py_object) {
  if (py_object.is_none()) {
    return {};
  }
  T* cpp_object = py::cast<T*>(py_object);
  return std::shared_ptr<T>(
      /* stored pointer = */ cpp_object,
      /* deleter = */ [captured_py_object = std::move(py_object)](
                          void*) mutable {
        py::gil_scoped_acquire deleter_guard;
        captured_py_object = py::none();
      });
}

}  // namespace pydrake
}  // namespace drake

/// Allow numpy arrays of with dtype=object containing `Type` objects to convert
/// to and from Eigen matrices of `Type`.
#define PYDRAKE_NUMPY_OBJECT_DTYPE(Type) PYBIND11_NUMPY_OBJECT_DTYPE(Type)

// Legacy synonym for PYDRAKE_NUMPY_OBJECT_DTYPE. Don't use this in new code.
#define DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(Type) \
  PYBIND11_NUMPY_OBJECT_DTYPE(Type)

// This alias helps ease Drake's transition to nanobind.
#define PYDRAKE_MODULE PYBIND11_MODULE
