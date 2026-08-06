#pragma once

#include <memory>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>

// Here we include a lot of the pybind11 (or nanobind) API, to ensure that all
// code in pydrake sees the same definitions ("One Definition Rule") for
// template types intended for specialization. Any headers with `type_caster<>`
// specializations must be included here (e.g., eigen.h, functional.h, numpy.h,
// stl.h) as well as ADL headers (e.g., operators.h). Headers that are unused by
// pydrake (e.g., complex.h) are omitted.
#ifdef PYDRAKE_USE_PYBIND11
#include "pybind11/eigen.h"
#include "pybind11/eval.h"
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
#include "drake_nanobind/eigen/dense.h"
#include "drake_nanobind/eigen/sparse.h"
#include "nanobind/eval.h"
#include "nanobind/make_iterator.h"
#include "nanobind/nanobind.h"
#include "nanobind/ndarray.h"
#include "nanobind/operators.h"
#include "nanobind/stl/array.h"
#include "nanobind/stl/filesystem.h"
#include "nanobind/stl/function.h"
#include "nanobind/stl/list.h"
#include "nanobind/stl/map.h"
#include "nanobind/stl/optional.h"
#include "nanobind/stl/pair.h"
#include "nanobind/stl/set.h"
#include "nanobind/stl/shared_ptr.h"
#include "nanobind/stl/string.h"
#include "nanobind/stl/string_view.h"
#include "nanobind/stl/tuple.h"
#include "nanobind/stl/unique_ptr.h"
#include "nanobind/stl/unordered_map.h"
#include "nanobind/stl/unordered_set.h"
#include "nanobind/stl/variant.h"
#include "nanobind/stl/vector.h"
#include "nanobind/trampoline.h"
#pragma GCC diagnostic pop

#include "drake/bindings/pydrake/numpy_object_pybind.h"
#include "drake/bindings/pydrake/reference_wrapper_pybind.h"
#endif  // PYDRAKE_USE_NANOBIND

namespace drake {

/// For more high-level information, see the @ref python_bindings
/// "Python Bindings" technical notes.
///
/// Drake developers should prefer any aliases defined here over their full
/// spellings in `pybind11` or `nanobind`.
///
/// `namespace py` is a shorthand alias to either `pybind11` or `nanobind`, for
/// consistency. (This symbol cannot be exposed directly in Doxygen.)
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

/// Shortened alias for py::rv_policy. For more information, see
/// the @ref PydrakeReturnValuePolicy "Return Value Policy" section.
using py_rvp = py::rv_policy;

// This alias helps ease Drake's transition to nanobind.
#ifdef PYDRAKE_USE_PYBIND11
using py::class_;
#else   // PYDRAKE_USE_NANOBIND
namespace internal {
// XXX need locking?
class AliasRegistry {
 public:
  static void AddAlias(
      const ::std::type_info& alias_type, const ::std::type_info& bound_type) {
    (*TheMap())[std::type_index(alias_type)] = &bound_type;
  }

  static const std::type_info* Unalias(const std::type_info* query) {
    auto& unalias = *TheMap();
    auto it = unalias.find(std::type_index(*query));
    if (it == unalias.end()) {
      return query;
    }
    return it->second;
  }

 private:
  using UnaliasMap = std::unordered_map<std::type_index, const std::type_info*>;
  static UnaliasMap* TheMap() {
    py::module_ m = py::module_::import_("pydrake.common.alias_registry");
    py::capsule py_map = m.attr("_unalias_map");
    UnaliasMap* cpp_map{};
    if (py_map.is_none()) {
      // Purposefully never destroyed; no cleanup routine is provided.
      py_map = py::capsule(new UnaliasMap);
      m.attr("_unalias_map") = py_map;
    }
    cpp_map = static_cast<UnaliasMap*>(py_map.data());
    return cpp_map;
  }
};

// We use partial template specialiation of a traits-like type to drop the
// shared_ptr holder type annotation on py::class_ declarations. Only pybind11
// uses holder types in the class_ template argument list.
template <typename T, typename... Ts>
struct PyClassRemoveSharedPtrHolderAnnotation {
  using type = py::class_<T, Ts...>;
};
template <typename T, typename Holder, typename... Ts>
struct PyClassRemoveSharedPtrHolderAnnotation<T, Holder, Ts...> {
  using type = std::conditional_t<std::is_same_v<Holder, std::shared_ptr<T> >,
      py::class_<T, Ts...>, py::class_<T, Holder, Ts...> >;
};
template <typename T, typename Base, typename Holder, typename... Ts>
struct PyClassRemoveSharedPtrHolderAnnotation<T, Base, Holder, Ts...> {
  using type = std::conditional_t<std::is_same_v<Holder, std::shared_ptr<T> >,
      py::class_<T, Base, Ts...>, py::class_<T, Base, Holder, Ts...> >;
};
template <typename T, typename Base1, typename Base2, typename Holder,
    typename... Ts>
struct PyClassRemoveSharedPtrHolderAnnotation<T, Base1, Base2, Holder, Ts...> {
  using type = std::conditional_t<std::is_same_v<Holder, std::shared_ptr<T> >,
      py::class_<T, Base1, Base2, Ts...>,
      py::class_<T, Base1, Base2, Holder, Ts...> >;
};

}  // namespace internal

template <typename T, typename... Ts>
class __attribute__((visibility("hidden"))) class_
    : public internal::PyClassRemoveSharedPtrHolderAnnotation<T, Ts...>::type {
 public:
  using Base = internal::PyClassRemoveSharedPtrHolderAnnotation<T, Ts...>::type;
  explicit class_(auto&&... args)
      : Base(std::forward<decltype(args)>(args)...,
            py::is_weak_referenceable()) {
    if constexpr (!std::is_same_v<T, typename Base::Alias>) {
      internal::AliasRegistry::AddAlias(
          typeid(typename Base::Alias), typeid(T));
    }
  }
};
#endif  // PYDRAKE_USE_PYBIND11

namespace internal {
#ifdef PYDRAKE_USE_PYBIND11
template <typename T>
using is_pyobject = py::detail::is_pyobject<T>;
#else   // PYDRAKE_USE_NANOBIND
template <typename T>
using is_pyobject =
    std::is_base_of<py::detail::api_tag, std::remove_reference_t<T> >;
#endif  // PYDRAKE_USE_PYBIND11
}  // namespace internal

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
/// return value.
template <typename PyClass, typename GetState, typename SetState>
void DefPickle(PyClass* ppy_class, GetState&& get_state, SetState&& set_state) {
  PyClass& py_class = *ppy_class;
#ifdef PYDRAKE_USE_NANOBIND
  py_class.def("__getstate__", std::forward<GetState>(get_state));
  py_class.def("__setstate__", std::forward<SetState>(set_state));
#else   // PYDRAKE_USE_PYBIND11
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
#endif  // PYDRAKE_USE_NANOBIND
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
#ifdef PYDRAKE_USE_PYBIND11
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
#else   // PYDRAKE_USE_NANOBIND
template <typename CppClass>
struct __attribute__((visibility("hidden"))) ParamInit
    : py::def_visitor<ParamInit<CppClass> > {
  template <typename PyClass, typename... Extra>
  void execute(PyClass& cl, const Extra&...) {
    cl.def("__init__", [](CppClass* self, py::kwargs kwargs) {
      new (self) CppClass();
      py::object py_obj = py::cast(self, py_rvp::reference);

      // Nanobind wouldn't have known the C++ instance is ready yet, but we have
      // to mark it ready to allow all of the setattr machinery to work before
      // init returns. This also set the object's `destruct` flag to true, so
      // that if the _setattr_kwargs raises, `self`'s C++ destructor will run.
      py::inst_mark_ready(py_obj);

      py::module_::import_("pydrake").attr("_setattr_kwargs")(py_obj, kwargs);
    });
  }
};
#endif  // PYDRAKE_USE_PYBIND11

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
#ifdef PYDRAKE_USE_PYBIND11
#define PYDRAKE_NUMPY_OBJECT_DTYPE(Type) PYBIND11_NUMPY_OBJECT_DTYPE(Type)
#endif  // PYDRAKE_USE_PYBIND11
// N.B. For PYDRAKE_USE_NANOBIND the numpy_object_pybind.h has already defined
// the PYDRAKE_NUMPY_OBJECT_DTYPE macro.

#ifdef PYDRAKE_USE_PYBIND11
// Legacy synonym for PYDRAKE_NUMPY_OBJECT_DTYPE. Don't use this in new code.
#define DRAKE_PYBIND11_NUMPY_OBJECT_DTYPE(Type) \
  PYBIND11_NUMPY_OBJECT_DTYPE(Type)
#endif  // PYDRAKE_USE_PYBIND11

// These aliases help ease Drake's transition to nanobind.
#ifdef PYDRAKE_USE_PYBIND11
#define PYDRAKE_MODULE PYBIND11_MODULE
#define PYDRAKE_BINDER_NAMESPACE pybind11
#define PYDRAKE_OVERRIDE PYBIND11_OVERRIDE
#define PYDRAKE_OVERRIDE_PURE PYBIND11_OVERRIDE_PURE
// This is an implementation of nanobind's NB_TRAMPOLINE macro for pybind11.
// https://nanobind.readthedocs.io/en/latest/classes.html#overriding-virtual-functions-in-python
// In particular, `size` should match how many PYDRAKE_OVERRIDE{,_PURE} are used
// within the class.
#define NB_TRAMPOLINE(base, size) \
  static_assert(size >= 0);       \
  using NBBase = base;            \
  using NBBase::NBBase
#else  // PYDRAKE_USE_NANOBIND
#define PYDRAKE_MODULE NB_MODULE
#define PYDRAKE_BINDER_NAMESPACE nanobind
#define PYDRAKE_OVERRIDE(unused1, unused2, func, ...)                         \
  do {                                                                        \
    try {                                                                     \
      NB_OVERRIDE(func, __VA_ARGS__);                                         \
    } catch (const py::builtin_exception& e) {                                \
      /* In case the method was not overridden, nanobind might erroneously */ \
      /* throw instead of using the base class implementation. This will */   \
      /* happen when the C++ base class method isn't bound in pydrake. */     \
      /* We'll check for that exact failure mode and handle it here. */       \
      if (e.type() == py::exception_type::runtime_error) {                    \
        const std::string_view what = e.what();                               \
        if (what.starts_with("nanobind::detail::get_trampoline('") &&         \
            what.ends_with("'): lookup failed!")) {                           \
          { /* Flush the failure from PyObject_GetAttr. */                    \
            py::gil_scoped_acquire guard;                                     \
            PyErr_Clear();                                                    \
          }                                                                   \
          return NBBase::func(__VA_ARGS__);                                   \
        }                                                                     \
      }                                                                       \
      throw;                                                                  \
    }                                                                         \
  } while (0)
#define PYDRAKE_OVERRIDE_PURE(unused1, unused2, ...) \
  NB_OVERRIDE_PURE(__VA_ARGS__)
#endif  // PYDRAKE_USE_PYBIND11
