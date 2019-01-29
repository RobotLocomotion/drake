#pragma once

#include <array>
#include <map>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/operators.h>

#include "drake/bindings/pybind11_ext/numpy_ufunc.h"

// N.B. For NumPy dtypes, `custom` tends to mean record-like structures, while
// `user-defined` means teaching NumPy about previously opaque C structures.

// TODO(eric.cousineau): Figure out how to make this automatically hidden.
#pragma GCC visibility push(hidden)

namespace pybind11 {
namespace detail {

// The following code effectively creates a separate instance system than what
// pybind11 nominally has. This is done because, at present, it's difficult to
// have pybind11 extend other python types, in this case, `np.generic` /
// `PyGenericArrType_Type` (#1170).

// TODO(eric.cousineau): Get rid of this structure if #1170 can be resolved.

typedef PyObject* (*nb_conversion_t)(PyObject*);

// Stores dtype-specific information, as well as static access to relevant
// internals.
// Effectively a watered down version of `detail::type_info`.
struct dtype_info {
  handle cls;
  int dtype_num{-1};
  std::map<void*, PyObject*> instance_to_py;
  std::vector<type_info::implicit_conversion_func> implicit_conversions;
  std::map<std::type_index, nb_conversion_t> nb_implicit_conversions;

  // Provides mutable entry for a registered type, with option to create.
  template <typename T>
  static dtype_info& get_mutable_entry(bool is_new = false) {
    auto& internals = get_mutable_internals();
    std::type_index id(typeid(T));
    if (is_new) {
      if (internals.find(id) != internals.end())
        pybind11_fail("Class already registered");
      return internals[id];
    } else {
      return internals.at(id);
    }
  }

  // Provides immutable entry for a registered type.
  template <typename T>
  static const dtype_info& get_entry() {
    return get_mutable_internals().at(std::type_index(typeid(T)));
  }

  // Provides immutable entry for a registered type, given the typeid.
  static const dtype_info& get_entry(std::type_index id) {
    return get_mutable_internals().at(id);
  }

  // Provides immutable entry for a registered type, or nullptr.
  static const dtype_info* maybe_get_entry(std::type_index id) {
    const auto& internals = get_mutable_internals();
    auto iter = internals.find(id);
    if (iter != internals.end()) {
      return &iter->second;
    } else {
      return nullptr;
    }
  }

  // Finds the corresponding typeid for a `cls`, return `nullptr` if nothing is
  // found.
  static const std::type_index* find_entry(object cls) {
    auto& map = get_internals();
    for (auto& iter : map) {
      auto& entry = iter.second;
      if (cls.ptr() == entry.cls.ptr())
        return &iter.first;
    }
    return nullptr;
  }

 private:
  using internals = std::map<std::type_index, dtype_info>;
  static const internals& get_internals() {
    return get_mutable_internals();
  }

  // TODO(eric.cousineau): Store in internals.
  static internals& get_mutable_internals() {
    static internals* ptr =
        &get_or_create_shared_data<internals>("_numpy_dtype_user_internals");
    return *ptr;
  }
};

// CPython extension of `PyObject` for per-instance information of a
// user-defind dtype. Akin to `detail::instance`.
template <typename Class>
struct dtype_user_instance {
  PyObject_HEAD
  // TODO(eric.cousineau): Consider storing a unique_ptr to reduce the number
  // of temporaries.
  Class value;

  // Extracts C++ pointer from a given python object. No type checking is done.
  static Class* load_raw(PyObject* src) {
    dtype_user_instance* obj = reinterpret_cast<dtype_user_instance*>(src);
    return &obj->value;
  }

  // Allocates an instance.
  static dtype_user_instance* alloc_py() {
    auto cls = dtype_info::get_entry<Class>().cls;
    PyTypeObject* cls_raw = reinterpret_cast<PyTypeObject*>(cls.ptr());
    auto obj = reinterpret_cast<dtype_user_instance*>(
        cls_raw->tp_alloc(cls_raw, 0));
    // Ensure we clear out the memory.
    memset(&obj->value, 0, sizeof(Class));
    return obj;
  }

  // Implementation for `tp_new` slot.
  static PyObject* tp_new(
      PyTypeObject* /*type*/, PyObject* /*args*/, PyObject* /*kwds*/) {
    // N.B. `__init__` should call the in-place constructor.
    auto obj = alloc_py();
    // // Register.
    auto& entry = dtype_info::get_mutable_entry<Class>();
    PyObject* pyobj = reinterpret_cast<PyObject*>(obj);
    entry.instance_to_py[&obj->value] = pyobj;
    return pyobj;
  }

  // Implementation for `tp_dealloc` slot.
  static void tp_dealloc(PyObject* pself) {
    Class* value = load_raw(pself);
    // Call destructor.
    value->~Class();
    // Deregister.
    auto& entry = dtype_info::get_mutable_entry<Class>();
    entry.instance_to_py.erase(value);
  }

  // Instance finding. Returns empty `object` if nothing is found.
  static object find_existing(const Class* value) {
    auto& entry = dtype_info::get_entry<Class>();
    void* raw = const_cast<Class*>(value);
    auto it = entry.instance_to_py.find(raw);
    if (it == entry.instance_to_py.end()) {
      return {};
    } else {
      return reinterpret_borrow<object>(it->second);
    }
  }
};

// Implementation of `type_caster` to interface `dtype_user_instance<>`s.
template <typename Class>
struct dtype_user_caster {
  static constexpr auto name = detail::_<Class>();
  using DTypePyObject = dtype_user_instance<Class>;

  // Casts a const lvalue reference to a Python object.
  static handle cast(const Class& src, return_value_policy, handle) {
    object h = DTypePyObject::find_existing(&src);
    // TODO(eric.cousineau): Handle parenting?
    if (!h) {
      // Make new instance.
      DTypePyObject* obj = DTypePyObject::alloc_py();
      obj->value = src;
      h = reinterpret_borrow<object>(reinterpret_cast<PyObject*>(obj));
      return h.release();
    }
    return h.release();
  }

  // Casts a pointer to a Python object.
  static handle cast(const Class* src, return_value_policy policy, handle) {
    object h = DTypePyObject::find_existing(src);
    if (h) {
      return h.release();
    } else {
      if (policy == return_value_policy::automatic_reference ||
          policy == return_value_policy::reference) {
        throw cast_error("Cannot find existing instance");
      } else {
        // Copy the instance.
        DTypePyObject* obj = DTypePyObject::alloc_py();
        obj->value = *src;
        delete src;
        h = reinterpret_borrow<object>(reinterpret_cast<PyObject*>(obj));
        return h.release();
      }
    }
  }

  // Load from Python to C++. The result will be retrieved by the casting
  // operators below.
  bool load(handle src, bool convert) {
    auto& entry = dtype_info::get_entry<Class>();
    auto cls = entry.cls;
    object obj;
    if (!isinstance(src, cls)) {
      // Check if it's an `np.array` with matching dtype.
      handle array = reinterpret_cast<PyObject*>(
          npy_api::get().PyArray_Type_);
      if (isinstance(src, array)) {
        tuple shape = src.attr("shape");
        if (shape.size() == 0) {
          obj = src.attr("item")();
        }
      }
      if (!obj && convert) {
        // Try implicit conversions.
        for (auto& converter : entry.implicit_conversions) {
          auto temp = converter(
              src.ptr(), reinterpret_cast<PyTypeObject*>(cls.ptr()));
          if (temp) {
            obj = reinterpret_steal<object>(temp);
            loader_life_support::add_patient(obj);
            break;
          }
        }
      }
    } else {
      obj = reinterpret_borrow<object>(src);
    }
    if (!obj) {
      return false;
    } else {
      ptr_ = DTypePyObject::load_raw(obj.ptr());
      return true;
    }
  }

  // Copy `type_caster_base`.
  template <typename T_> using cast_op_type =
      pybind11::detail::cast_op_type<T_>;

  // Retrieves result after `load()`.
  operator Class&() { return *ptr_; }
  // Retrieves result after `load()`.
  operator Class*() { return ptr_; }

 private:
  // Stores result after `load()`.
  Class* ptr_{};
};

// Ensures that `dtype_user_caster` can cast pointers. See `cast.h`.
template <typename T>
struct cast_is_known_safe<T,
    enable_if_t<std::is_base_of<
        dtype_user_caster<intrinsic_t<T>>, make_caster<T>>::value>>
    : public std::true_type {};

// Maps a common Python function name to a NumPy UFunc name, or just returns
// the original name (for trigonometric functions).
inline std::string get_ufunc_name(std::string name) {
  static const std::map<std::string, const char*> m = {
    // Anything that is mapped to `nullptr` implies that NumPy does not support
    // this ufunc.
    // https://docs.python.org/2.7/reference/datamodel.html#emulating-numeric-types  // NOLINT(whitespace/line_length)
    // Use nominal ordering (e.g. `__add__`, not `__radd__`) as ordering will
    // be handled by ufunc registration.
    // Use Python 3 operator names (e.g. `__truediv__`)
    // https://docs.scipy.org/doc/numpy/reference/routines.math.html
    {"__add__", "add"},
    {"__iadd__", nullptr},
    {"__neg__", "negative"},
    {"__pos__", nullptr},
    {"__mul__", "multiply"},
    {"__imul__", nullptr},
    // https://docs.scipy.org/doc/numpy/reference/routines.bitwise.html
    {"__and__", "bitwise_and"},
    {"__iand__", nullptr},
    {"__or__", "bitwise_or"},
    {"__ior__", nullptr},
    {"__xor__", "bitwise_xor"},
    {"__ixor__", nullptr},
    // TODO(eric.cousineau): Figure out how to appropriately map `true_divide`
    // vs. `divide` when the output type is adjusted?
    {"__truediv__", "divide"},
    {"__itruediv__", nullptr},
    {"__pow__", "power"},
    {"__sub__", "subtract"},
    {"__isub__", nullptr},
    {"__abs__", "absolute"},
    // https://docs.scipy.org/doc/numpy/reference/routines.logic.html
    {"__gt__", "greater"},
    {"__ge__", "greater_equal"},
    {"__lt__", "less"},
    {"__le__", "less_equal"},
    {"__eq__", "equal"},
    {"__ne__", "not_equal"},
    {"__bool__", "nonzero"},  // Python3
    {"__nonzero__", "nonzero"},  // Python2.7
    {"__invert__", "logical_not"},
    // Are these necessary?
    {"min", "fmin"},
    {"max", "fmax"},
    // TODO(eric.cousineau): Add something for junction-style logic?
  };
  auto iter = m.find(name);
  if (iter != m.end()) {
    if (!iter->second) {
      throw std::runtime_error("Invalid NumPy operator: " + name);
    }
    return iter->second;
  } else {
    return name;
  }
}

// Provides implementation of `npy_format_decsriptor` for a user-defined dtype.
template <typename Class>
struct dtype_user_npy_format_descriptor {
    static constexpr auto name = detail::_<Class>();
    static pybind11::dtype dtype() {
        int dtype_num = dtype_info::get_entry<Class>().dtype_num;
        if (auto ptr = detail::npy_api::get().PyArray_DescrFromType_(dtype_num))
            return reinterpret_borrow<pybind11::dtype>(ptr);
        pybind11_fail("Unsupported buffer format!");
    }
};

// Stores information about a conversion.
template <typename From, typename To, typename Func>
struct dtype_conversion_t {
  Func func;
  bool allow_implicit_coercion{};
};

// Infers the correct signature for `dtype_conversion_t` from a function.
template <typename FuncIn>
static auto dtype_conversion_impl(
    FuncIn&& func_in, bool allow_implicit_coercion) {
  auto func_infer = detail::infer_function_info(func_in);
  using FuncInfer = decltype(func_infer);
  using From = detail::intrinsic_t<
      typename FuncInfer::Args::template type_at<0>>;
  using To = detail::intrinsic_t<typename FuncInfer::Return>;
  using Func = typename FuncInfer::Func;
  return dtype_conversion_t<From, To, Func>{
      std::forward<Func>(func_infer.func), allow_implicit_coercion};
}

}  // namespace detail

/// Provides user control over definition of UFuncs.
struct dtype_method {
  /// Defines `np.dot` for a given type.
  struct dot {};

  /// Uses constructor / casting for explicit conversion.
  template <typename From, typename To>
  static auto explicit_conversion() {
    return detail::dtype_conversion_impl([](const From& in) {
      return To(in);
    }, false);
  }

  /// Provides function for explicit conversion.
  template <typename Func>
  static auto explicit_conversion(Func&& func) {
    return detail::dtype_conversion_impl(std::forward<Func>(func), false);
  }

  /// Uses constructor / casting for implicit conversion.
  template <typename From, typename To>
  static auto implicit_conversion() {
    return detail::dtype_conversion_impl(
        [](const From& in) -> To { return in; }, true);
  }

  /// Provides function for implicit conversion.
  template <typename Func>
  static auto implicit_conversion(Func&& func) {
    return detail::dtype_conversion_impl(std::forward<Func>(func), true);
  }

  /// Implies that only a `ufunc` should be defined, and the corresponding class
  /// method should not be defined.
  struct ufunc_only {};
};

/**
Defines a user-defined dtype.

Constraints:
* The type must be copy-constructible and assignable.
* The type *may* not have its constructor called; however, its memory *will* be
initialized to zero, so it's assignment should be robust against being assigned
from zero memory.
* This type's instance *won't* always be destroyed, because NumPy does not have
slots to define this yet.
 */
template <typename Class_>
class dtype_user : public object {
 public:
  static_assert(
      !std::is_polymorphic<Class_>::value,
      "Cannot define NumPy dtypes for polymorphics classes.");

  using PyClass = class_<Class_>;
  using type = typename PyClass::type;
  using Class = Class_;
  using DTypePyObject = detail::dtype_user_instance<Class>;

  dtype_user(handle scope, const char* name, const char* doc = "")
      : cls_(none()) {
    register_type(name, doc);
    scope.attr(name) = self();
    auto& entry = detail::dtype_info::get_mutable_entry<Class>(true);
    entry.cls = self();
    // Register numpy type.
    // (Note that not registering the type will result in infinte recursion).
    entry.dtype_num = register_numpy();

    // Register default ufunc cast to `object`.
    // N.B. Given how general this is, it should *NEVER* be implicit, as it
    // would interfere with more meaningful casts.
    // N.B. This works because `object` is defined to have the same memory
    // layout as `PyObject*`, thus can be registered in lieu of `PyObject*` -
    // this also effectively increases the refcount and releases the object.
    this->def_loop(dtype_method::explicit_conversion(
        [](const Class& self) -> object { return pybind11::cast(self); }));
    object cls = self();
    auto object_to_cls = [cls](object obj) -> Class {
      // N.B. We use the *constructor* rather than implicit conversions because
      // implicit conversions may not be sufficient when dealing with `object`
      // dtypes. As an example, a class can only explicitly cast to float, but
      // the array is constructed as `np.array([1., Class(2)])`. The inferred
      // dtype in this case will be `object`.
      if (!isinstance(obj, cls)) {
        // This will catch type mismatch errors.
        // TODO(eric.cousineau): Not having the correct constructor registered
        // can causes segfaults when the error is begin printed out, due to the
        // indirection of `_dtype_init`. Consider changing this...
        obj = cls(obj);
      }
      return obj.cast<Class>();
    };
    this->def_loop(dtype_method::explicit_conversion(object_to_cls));
  }

  ~dtype_user() {
    // This will be called once the `pybind11` module ends, and thus should
    // warn the user if they've left this class in a bad state.
    check();
  }

  /// Forwards method definition to `py::class_`.
  template <typename ... Args>
  dtype_user& def(const char* name, Args&&... args) {
    cls().def(name, std::forward<Args>(args)...);
    return *this;
  }

  /// Defines a constructor.
  template <typename ... Args, typename ... Extra>
  dtype_user& def(
      detail::initimpl::constructor<Args...>&&, Extra&&... extra) {
    // See notes in `add_init`.
    // N.B. Do NOT use `Class*` as the argument, since that may incur recursion.
    add_init([](object py_self, Args... args) {
      // Old-style. No factories for now.
        Class* self = DTypePyObject::load_raw(py_self.ptr());
      new (self) Class(std::forward<Args>(args)...);
    }, detail::type_pack<Args...>{}, std::forward<Extra>(extra)...);
    return *this;
  }

  /// Defines UFunc loop operator.
  template <detail::op_id id, detail::op_type ot,
      typename L, typename R>
  dtype_user& def_loop(
      const detail::op_<id, ot, L, R>&, dtype_method::ufunc_only) {
    // Register ufunction with builtin name.
    // Use `op_l`. Mapping `__radd__` to `add` would require remapping argument
    // order, and screw that. We can just use the fact that `op_impl` is
    // generic.
    constexpr auto ot_norm = (ot == detail::op_r) ? detail::op_l : ot;
    using op_norm_ = detail::op_<id, ot_norm, L, R>;
    using op_norm_impl = typename op_norm_::template info<PyClass>::op;
    std::string ufunc_name = detail::get_ufunc_name(op_norm_impl::name());
    ufunc::get_builtin(ufunc_name.c_str())  // BR
        .def_loop<Class>(&op_norm_impl::execute);
    if (ufunc_name == "divide") {
      ufunc::get_builtin("true_divide").def_loop<Class>(&op_norm_impl::execute);
    }
    return *this;
  }

  /// Defines Python and UFunc loop operator.
  template <detail::op_id id, detail::op_type ot,
      typename L, typename R>
  dtype_user& def_loop(const detail::op_<id, ot, L, R>& op) {
    // Define Python class operator.
    using op_ = detail::op_<id, ot, L, R>;
    using op_impl = typename op_::template info<PyClass>::op;
    this->def(op_impl::name(), &op_impl::execute, is_operator());
    // Define dtype operators.
    return def_loop(op, dtype_method::ufunc_only());
  }

  /// Defines a scalar function and overloads an existing NumPy UFunc loop,
  /// mapping to a buitlin name in `numpy`.
  template <typename Func>
  dtype_user& def_loop(const char* name, const Func& func) {
    cls().def(name, func);
    std::string ufunc_name = detail::get_ufunc_name(name);
    ufunc::get_builtin(ufunc_name.c_str()).def_loop<Class>(func);
    return *this;
  }

  /// Forwards operator defintiion to `py::class_`.
  template <detail::op_id id, detail::op_type ot,
      typename L, typename R, typename... Extra>
  dtype_user& def(
      const detail::op_<id, ot, L, R>& op, const Extra&... extra) {
    cls().def(op, extra...);
    return *this;
  }

  /// Defines loop cast, and optionally permit implicit conversions.
  template <typename From, typename To, typename Func>
  dtype_user& def_loop(
      detail::dtype_conversion_t<From, To, Func> conv,
      dtype from = {}, dtype to = {}) {
    detail::ufunc_register_cast<From, To>(
        conv.func, conv.allow_implicit_coercion, from, to);
    // Define implicit conversion on the class.
    if (conv.allow_implicit_coercion) {
      if (std::is_same<To, Class>::value) {
        // TODO(eric.cousineau): Is this a good idea? It's quite confusing to
        // discard the function here.
        auto& entry = detail::dtype_info::get_mutable_entry<Class>();
        entry.implicit_conversions.push_back(
            detail::create_implicit_caster<From, Class>());
      } else {
        auto enabled = std::is_same<From, Class>{};
        register_nb_conversion<To>(enabled, conv.func);
      }
    }
    return *this;
  }

  /// Defines dot product.
  template <typename Defer = void>
  dtype_user& def_loop(dtype_method::dot) {
    // TODO(eric.cousineau): See if there is a way to define `dot` for an
    // algebra that is not closed under addition / multiplication (e.g.
    // symbolic variable -> symbolic expression).
    if (arrfuncs_->dotfunc)
      pybind11_fail("dtype: Cannot redefine `dot`");
    using detail::npy_intp;
    arrfuncs_->dotfunc = reinterpret_cast<void*>(+[](
        void* ip0_, npy_intp is0, void* ip1_, npy_intp is1,
        void* op, npy_intp n, void* /*arr*/) {
      const char *ip0 = reinterpret_cast<char*>(ip0_);
      const char *ip1 = reinterpret_cast<char*>(ip1_);
      Class r{};
      for (npy_intp i = 0; i < n; i++) {
        const Class& v1 = *reinterpret_cast<const Class*>(ip0);
        const Class& v2 = *reinterpret_cast<const Class*>(ip1);
          r += v1 * v2;
          ip0 += is0;
          ip1 += is1;
        }
        *reinterpret_cast<Class*>(op) = r;
    });
    return *this;
  }

  /// Access a `py::class_` view of the type. Please be careful when adding
  /// methods or attributes, as they may conflict with how NumPy works.
  PyClass& cls() { return cls_; }

 private:
  // Provides mutable explicit upcast reference (for assignment).
  object& self() { return *this; }
  const object& self() const { return *this; }

  // Checks definition invariants.
  void check() const {
    auto warn = [](const std::string& msg) {
      // TODO(eric.cousineau): Figure out better warning type.
      PyErr_WarnEx(PyExc_UserWarning, msg.c_str(), 0);
    };
    // This `dict` should indicate whether we've directly overridden methods.
    dict d = self().attr("__dict__");
    // Without these, numpy goes into infinite recursion. Haven't bothered to
    // figure out exactly why.
    if (!d.contains("__repr__"))
      warn("dtype: Class is missing explicit __repr__!");
    if (!d.contains("__str__"))
      warn("dtype: Class is missing explicit __str__!");
  }

  // Adds constructor. See comments within for explanation.
  template <typename Func, typename... Extra, typename... Args>
  void add_init(Func&& f, detail::type_pack<Args...>, Extra&&... extra) {
    // Do not construct this with the name `__init__` as `pybind11`s
    // constructor implementations via `cpp_function` are rigidly fixed to
    // its instance registration system (which we don't want).
    // Because of this, if there is an error in constructors when testing
    // overloads, `repr()` may be called on an object in an invalid state.
    this->def("_dtype_init", std::forward<Func>(f));
    // Ensure that this is called by a non-pybind11-instance `__init__`.
    dict d = self().attr("__dict__");
    if (!d.contains("__init__")) {
      auto init = self().attr("_dtype_init");
      auto func = cpp_function(
          [init](handle self, Args... args) {
            // Dispatch.
            init(self, std::forward<Args>(args)...);
          }, is_method(self()), std::forward<Extra>(extra)...);
      self().attr("__init__") = func;
    }
  }

  // Handles conversions from `nb_*` methods in Python type objects. Uses
  // available conversions if possible, otherwise will cause an error to be
  // thrown when called.
  template <typename T>
  static PyObject* handle_nb_conversion(PyObject* from) {
    auto& entry = detail::dtype_info::get_entry<Class>();
    auto& conversions = entry.nb_implicit_conversions;
    // Check for available conversions.
    std::type_index id(typeid(T));
    auto iter = conversions.find(id);
    if (iter != conversions.end()) {
      return iter->second(from);
    } else {
      PyErr_SetString(
        PyExc_TypeError,
        "dtype_user: Direct casting via Python not supported");
      return nullptr;
    }
  }

  // Handles registering native `nb_*` type conversions.
  template <typename To, typename Func>
  void register_nb_conversion(std::true_type, const Func& func) {
    auto& entry = detail::dtype_info::get_mutable_entry<Class>();
    std::type_index id(typeid(To));
    auto& conversions = entry.nb_implicit_conversions;
    assert(conversions.find(id) == conversions.end());
    static Func func_static = func;
    detail::nb_conversion_t nb_conversion =
        +[](PyObject* from_py) -> PyObject* {
          Class* from = pybind11::cast<Class*>(from_py);
          To to = func_static(*from);
          return pybind11::cast<To>(to).release().ptr();
        };
    conversions[id] = nb_conversion;
  }

  template <typename To, typename Func>
  void register_nb_conversion(std::false_type, const Func&) {}

  // Disables `nb_coerce`.
  static int disable_nb_coerce(PyObject**, PyObject**) {
    PyErr_SetString(
      PyExc_TypeError,
      "dtype_user: Direct coercion via Python not supported");
    return 1;
  }

  // Registers Python type.
  void register_type(const char* name, const char* doc) {
    // Ensure we initialize NumPy before accessing `PyGenericArrType_Type`.
    auto& api = detail::npy_api::get();
    // Loosely uses https://stackoverflow.com/a/12505371/7829525 as well.
    auto heap_type = reinterpret_cast<PyHeapTypeObject*>(
        PyType_Type.tp_alloc(&PyType_Type, 0));
    if (!heap_type)
        pybind11_fail("dtype_user: Could not register heap type");
    heap_type->ht_name = pybind11::str(name).release().ptr();
    // It's painful to inherit from `np.generic`, because it has no `tp_new`.
    auto& ClassObject_Type = heap_type->ht_type;
    ClassObject_Type.tp_base = api.PyGenericArrType_Type_;
    ClassObject_Type.tp_new = &DTypePyObject::tp_new;
    ClassObject_Type.tp_dealloc = &DTypePyObject::tp_dealloc;
    ClassObject_Type.tp_name = name;  // Er... scope?
    ClassObject_Type.tp_basicsize = sizeof(DTypePyObject);
    ClassObject_Type.tp_getset = 0;
    ClassObject_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_HEAPTYPE;
    ClassObject_Type.tp_doc = doc;
    if (PyType_Ready(&ClassObject_Type) != 0)
        pybind11_fail("dtype_user: Unable to initialize class");
    // TODO(eric.cousineau): Figure out how to catch recursions with
    // `tp_as_number` and casting, when it's not defined
    static auto tp_as_number = *ClassObject_Type.tp_as_number;
    ClassObject_Type.tp_as_number = &tp_as_number;
    // TODO(eric.cousineau): Figure out how to use more generic dispatch on
    // this object. If we use the `np.generic` stuff, we end up getting
    // recursive loops.
    tp_as_number.nb_float = &handle_nb_conversion<double>;
    tp_as_number.nb_int = &handle_nb_conversion<int>;
#if PY_VERSION_HEX < 0x03000000
    tp_as_number.nb_long = &handle_nb_conversion<int>;
    tp_as_number.nb_coerce = &disable_nb_coerce;
#endif
    // Create views into created type.
    self() = reinterpret_borrow<object>(
        reinterpret_cast<PyObject*>(&ClassObject_Type));
    cls_ = self();
  }

  // Registers NumPy dtype entry.
  int register_numpy() {
    using detail::npy_api;
    // Adapted from `numpy/core/multiarray/src/test_rational.c.src`.
    // Define NumPy description.
    auto type = reinterpret_cast<PyTypeObject*>(self().ptr());
    struct align_test { char c; Class r; };

    static detail::PyArray_ArrFuncs arrfuncs;
    static detail::PyArray_Descr descr = {
        PyObject_HEAD_INIT(0)
        type,                     /* typeobj */
        'V',                      /* kind (V = arbitrary) */
        'r',                      /* type */
        '=',                      /* byteorder */
        npy_api::NPY_NEEDS_PYAPI_ | npy_api::NPY_USE_GETITEM_ |
            npy_api::NPY_USE_SETITEM_ |
            npy_api::NPY_NEEDS_INIT_, /* flags */
        0,                        /* type_num */
        sizeof(Class),            /* elsize */
        offsetof(align_test, r),  /* alignment */
        0,                        /* subarray */
        0,                        /* fields */
        0,                        /* names */
        &arrfuncs,                /* f */
    };

    auto& api = npy_api::get();
    Py_TYPE(&descr) = api.PyArrayDescr_Type_;

    api.PyArray_InitArrFuncs_(&arrfuncs);

    using detail::npy_intp;

    // https://docs.scipy.org/doc/numpy/reference/c-api.types-and-structures.html
    arrfuncs.getitem = reinterpret_cast<void*>(
        +[](void* in, void* /*arr*/) -> PyObject* {
          auto item = reinterpret_cast<const Class*>(in);
          return pybind11::cast(*item).release().ptr();
        });
    arrfuncs.setitem = reinterpret_cast<void*>(
        +[](PyObject* in, void* out, void* /*arr*/) {
          detail::loader_life_support guard{};
          detail::dtype_user_caster<Class> caster;
          if (!caster.load(in, true)) {
            PyErr_SetString(
                PyExc_TypeError,
                "dtype_user: Could not convert during `setitem`");
            return -1;
          }
          *reinterpret_cast<Class*>(out) = caster;
          return 0;
        });
    arrfuncs.copyswap = reinterpret_cast<void*>(
        +[](void* dst, void* src, int swap, void* /*arr*/) {
          if (!src) return;
          Class* r_dst = reinterpret_cast<Class*>(dst);
          Class* r_src = reinterpret_cast<Class*>(src);
          if (swap) {
              PyErr_SetString(
                  PyExc_NotImplementedError,
                  "dtype_user: `swap` not implemented");
          } else {
              *r_dst = *r_src;
          }
        });
    arrfuncs.copyswapn = reinterpret_cast<void*>(
        +[](void* dst, npy_intp dstride, void* src,
            npy_intp sstride, npy_intp n, int swap, void*) {
          if (!src) return;
          if (swap) {
              PyErr_SetString(
                  PyExc_NotImplementedError,
                  "dtype_user: `swap` not implemented");
          } else {
              char* c_dst = reinterpret_cast<char*>(dst);
              char* c_src = reinterpret_cast<char*>(src);
              for (int k = 0; k < n; k++) {
                  Class* r_dst = reinterpret_cast<Class*>(c_dst);
                  Class* r_src = reinterpret_cast<Class*>(c_src);
                  *r_dst = *r_src;
                  c_dst += dstride;
                  c_src += sstride;
              }
          }
        });
    // - Ensure this doesn't overwrite our `equal` unfunc.
    arrfuncs.compare = reinterpret_cast<void*>(
        +[](const void* /*d1*/, const void* /*d2*/, void* /*arr*/) {
          pybind11_fail(
              "dtype: `compare` should not be called for pybind11 "
              "custom dtype");
        });
    arrfuncs.fillwithscalar = reinterpret_cast<void*>(
        +[](void* buffer_raw, npy_intp length, void* value_raw,
            void* /*arr*/) {
          const Class* value = reinterpret_cast<const Class*>(value_raw);
          Class* buffer = reinterpret_cast<Class*>(buffer_raw);
          for (int k = 0; k < length; k++) {
              buffer[k] = *value;
          }
          return 0;
        });
    int dtype_num = api.PyArray_RegisterDataType_(&descr);
    if (dtype_num < 0) {
      pybind11_fail("dtype_user: Could not register!");
    }
    self().attr("dtype") =
        reinterpret_borrow<object>(reinterpret_cast<PyObject*>(&descr));
    arrfuncs_ = &arrfuncs;
    return dtype_num;
  }

  PyClass cls_;
  detail::PyArray_ArrFuncs* arrfuncs_{};
};

}  // namespace pybind11

// Ensures that we can (a) cast the type (semi) natively, and (b) integrate
// with NumPy functionality.
#define PYBIND11_NUMPY_DTYPE_USER(Type)  \
    namespace pybind11 { namespace detail { \
      template <> \
      struct type_caster<Type> : public dtype_user_caster<Type> {}; \
      template <> \
      struct npy_format_descriptor<Type> \
          : public dtype_user_npy_format_descriptor<Type> {}; \
    }}

#pragma GCC visibility pop
