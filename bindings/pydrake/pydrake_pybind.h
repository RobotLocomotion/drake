#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#ifdef PYDRAKE_USE_PYBIND11
// Here we include a lot of the pybind11 API, to ensure that all code in pydrake
// sees the same definitions ("One Definition Rule") for template types intended
// for specialization. Any pybind11 headers with `type_caster<>` specializations
// must be included here (e.g., eigen.h, functional.h, numpy.h, stl.h) as well
// as ADL headers (e.g., operators.h). Headers that are unused by pydrake
// (e.g., complex.h) are omitted, as are headers that do not specialize anything
// (e.g., eval.h).
#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/stl/filesystem.h"
#include "pybind11/typing.h"
#endif  // PYDRAKE_USE_PYBIND11

#ifdef PYDRAKE_USE_NANOBIND
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wattributes"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wc++11-narrowing"
#else
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#pragma GCC diagnostic ignored "-Wnarrowing"
#endif  // __clang__
#include "nanobind/eigen/dense.h"
#include "nanobind/eval.h"
#include "nanobind/make_iterator.h"
#include "nanobind/nanobind.h"
#include "nanobind/ndarray.h"
#include "nanobind/operators.h"
#include "nanobind/stl/array.h"
#include "nanobind/stl/filesystem.h"
#include "nanobind/stl/function.h"
#include "nanobind/stl/map.h"
#include "nanobind/stl/optional.h"
#include "nanobind/stl/pair.h"
#include "nanobind/stl/set.h"
#include "nanobind/stl/shared_ptr.h"
#include "nanobind/stl/string.h"
#include "nanobind/stl/string_view.h"
#include "nanobind/stl/unique_ptr.h"
#include "nanobind/stl/unordered_map.h"
#include "nanobind/stl/variant.h"
#include "nanobind/stl/vector.h"
#include "nanobind/trampoline.h"
#pragma GCC diagnostic pop

#include "drake/common/autodiff.h"
#include "drake/common/drake_export.h"

// XXX porting shim-fest
namespace nanobind {
namespace detail {

template <typename T>
using is_pyobject = std::is_base_of<api_tag, std::remove_reference_t<T>>;

/** Add list-to-array implicit casting. */
template <typename... Args>
NB_INLINE ndarray_handle* ndarray_extra_import(PyObject* o,
    const ndarray_config*, bool, cleanup_list*,
    ndarray_config_t<int, Args...>*) noexcept {
  using Ndarray = ndarray<Args...>;
  using Config = ndarray_config_t<int, Args...>;
  // static constexpr bool ReadOnly = std::is_const_v<typename Config::Scalar>;
  using Scalar = std::remove_const<typename Config::Scalar>::type;
  // static constexpr char Order = Config::Order::value;
  // static constexpr int DeviceType = Config::DeviceType::value;
  // using VoidPtr = std::conditional_t<ReadOnly, const void *, void *>;
  if (!(PyList_Check(o) || PyTuple_Check(o))) {
    return nullptr;
  }
  // Build a data array for use with ndarray.
  size_t size = PySequence_Size(o);
  std::array<size_t, 2> shape{size, 1};
  if constexpr (std::is_floating_point_v<Scalar>) {
    auto data = std::make_unique<Scalar[]>(size);
    for (size_t k = 0; k < size; ++k) {
      data.get()[k] = static_cast<double>(float_(PySequence_GetItem(o, k)));
    }
    // XXX porting: double check memory management!
    Ndarray helper(data.release(), 2, shape.data());
    ndarray_handle* result = helper.handle();
    ndarray_inc_ref(result);
    return result;
  } else if constexpr (std::is_same_v<Scalar, drake::AutoDiffXd>) {
    auto data = std::make_unique<Scalar[]>(size);
    for (size_t k = 0; k < size; ++k) {
      data.get()[k] = cast<Scalar>(handle(PySequence_GetItem(o, k)));
    }
    // XXX porting: double check memory management!
    Ndarray helper(data.release(), 2, shape.data());
    ndarray_handle* result = helper.handle();
    ndarray_inc_ref(result);
    return result;
  } else {
    return nullptr;  // punt.
    // static_assert(false, "scalar trouble");
  }
}

}  // namespace detail
}  // namespace nanobind
#endif  // PYDRAKE_USE_NANOBIND

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
#ifdef PYDRAKE_USE_PYBIND11
namespace py = pybind11;
#else  // PYDRAKE_USE_NANOBIND
namespace py = nanobind;
#endif

/// Shortened alias for py::return_value_policy. For more information, see
/// the @ref PydrakeReturnValuePolicy "Return Value Policy" section.
using py_rvp = py::rv_policy;

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
#ifdef PYDRAKE_USE_PYBIND11
  using Class = typename PyClass::type;
#else  // PYDRAKE_USE_NANOBIND
  using Class = typename PyClass::Type;
#endif
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
#ifdef PYDRAKE_USE_PYBIND11
  using Class = typename PyClass::type;
#else  // PYDRAKE_USE_NANOBIND
  using Class = typename PyClass::Type;
#endif
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

/// Returns a constructor for creating an instance of Class and initializing
/// parameters (bound using `def_rw`).
/// This provides an alternative to manually enumerating each
/// parameter as an argument using `py::init<...>` and `py::arg(...)`, and is
/// useful when the C++ class only has a default constructor. Example:
/// @code
/// using Class = ExampleClass;
/// py::class_<Class>(m, "ExampleClass")  // BR
///     .def(ParamInit<Class>());
/// @endcode
///
/// @tparam Class The C++ class. Must have a default constructor.
template <typename CppClass>
struct DRAKE_NO_EXPORT ParamInit : py::def_visitor<ParamInit<CppClass>> {
  template <typename PyClass, typename... Extra>
  void execute(PyClass& cl, const Extra&...) {
    cl.def("__init__", [](CppClass* self, py::kwargs kwargs) {
      new (self) CppClass();
      py::object py_obj = py::cast(self, py_rvp::reference);

      // Nanobind wouldn't have known the c++ instance is ready yet, but we
      // have to mark it ready to allow all of the setattr machinery to work
      // before init returns.
      py::inst_mark_ready(py_obj);

      py::module_::import_("pydrake").attr("_setattr_kwargs")(py_obj, kwargs);
    });
  }
};

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

#ifdef PYDRAKE_USE_PYBIND11
#define DRAKE_NB_OBJECT_DTYPE(Type) \
  PYBIND11_NUMPY_OBJECT_DTYPE(Type)
#else  // PYDRAKE_USE_NANOBIND
// XXX porting needed
/*
*/
#define DRAKE_NB_NUMPY_OBJECT_DTYPE(Type)                       \
  namespace nanobind::detail {                                  \
  template <>                                                   \
  struct dtype_traits<Type> {                                   \
    static constexpr dlpack::dtype value{                       \
        static_cast<uint8_t>(dlpack::dtype_code::OpaqueHandle), \
        sizeof(object) * 8,                                     \
        1,                                                      \
    };                                                          \
    static constexpr auto name = const_name(#Type);             \
  };                                                            \
  }  // namespace nanobind::detail
#endif

// This alias helps ease Drake's transition to nanobind.
#ifdef PYDRAKE_USE_PYBIND11
#define PYDRAKE_MODULE PYBIND11_MODULE
#else  // PYDRAKE_USE_NANOBIND
#define PYDRAKE_MODULE NB_MODULE
#endif
