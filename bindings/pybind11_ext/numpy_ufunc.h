#pragma once

/// @file
/// Simple glue for NumPy UFuncs.

#include <memory>
#include <string>
#include <vector>

#include <pybind11/numpy.h>

#include "drake/bindings/pydrake/common/function_inference.h"
#include "drake/bindings/pydrake/common/type_pack.h"

// TODO(eric.cousineau): Figure out how to make this automatically hidden.
#pragma GCC visibility push(hidden)

namespace pybind11 {
namespace detail {

// Since this code lives in Drake, use existing headers.
using drake::type_pack;
using drake::type_pack_apply;
using drake::type_pack_concat;
using drake::pydrake::detail::infer_function_info;

// Utilities

// Builtins registered using
// numpy/build/{...}/numpy/core/include/numpy/__umath_generated.c

template <typename... Args>
struct ufunc_ptr {
  PyUFuncGenericFunction func{};
  void* data{};
};

// Unary ufunc.
template <typename Arg0, typename Out, typename Func>
auto ufunc_to_ptr(Func func, type_pack<Arg0, Out>) {
  auto ufunc = [](
      char** args, npy_intp* dimensions, npy_intp* steps, void* data) {
    Func& func_inner = *reinterpret_cast<Func*>(data);
    npy_intp step_0 = steps[0];
    npy_intp step_out = steps[1];
    npy_intp n = *dimensions;
    char *in_0 = args[0], *out = args[1];
    for (npy_intp k = 0; k < n; k++) {
      // TODO(eric.cousineau): Support pointers being changed.
      *reinterpret_cast<Out*>(out) = func_inner(*reinterpret_cast<Arg0*>(in_0));
      in_0 += step_0;
      out += step_out;
    }
  };
  // N.B. `new Func(...)` will never be destroyed.
  return ufunc_ptr<Arg0, Out>{ufunc, new Func(func)};
}

// Binary ufunc.
template <typename Arg0, typename Arg1, typename Out, typename Func = void>
auto ufunc_to_ptr(Func func, type_pack<Arg0, Arg1, Out>) {
  auto ufunc = [](
      char** args, npy_intp* dimensions, npy_intp* steps, void* data) {
    Func& func_inner = *reinterpret_cast<Func*>(data);
    npy_intp step_0 = steps[0];
    npy_intp step_1 = steps[1];
    npy_intp step_out = steps[2];
    npy_intp n = *dimensions;
    char *in_0 = args[0], *in_1 = args[1], *out = args[2];
    for (npy_intp k = 0; k < n; k++) {
      // TODO(eric.cousineau): Support pointers being fed in.
      *reinterpret_cast<Out*>(out) = func_inner(
          *reinterpret_cast<Arg0*>(in_0), *reinterpret_cast<Arg1*>(in_1));
      in_0 += step_0;
      in_1 += step_1;
      out += step_out;
    }
  };
  // N.B. `new Func(...)` will never be destroyed.
  return ufunc_ptr<Arg0, Arg1, Out>{ufunc, new Func(func)};
}

// Generic dispatch.
template <typename Func>
auto ufunc_to_ptr(Func func) {
  auto info = detail::infer_function_info(func);
  using Info = decltype(info);
  auto type_args = type_pack_apply<std::decay_t>(
    type_pack_concat(
      typename Info::Args{},
      type_pack<typename Info::Return>{}));
  return ufunc_to_ptr(func, type_args);
}

template <typename T>
void assert_ufunc_dtype_valid() {
  auto T_dtype = dtype::of<T>();
  int num = T_dtype.num();
  bool is_object = std::is_same<T, object>::value;
  if (num == npy_api::NPY_OBJECT_ && !is_object) {
  std::string message =
      "ufunc: Cannot handle `dtype=object` when T != `py::object` ";
  message += "(where T = " + type_id<T>() + "). ";
  message += "Please register function using `py::object`";
  pybind11_fail(message.c_str());
  }
}

template <typename From, typename To, typename Func>
void ufunc_register_cast(
  Func&& func, bool allow_coercion,
  dtype from = {}, dtype to = {}, type_pack<From, To> = {}) {
  assert_ufunc_dtype_valid<From>();
  assert_ufunc_dtype_valid<To>();
  static auto cast_lambda = detail::infer_function_info(func).func;
  auto cast_func = +[](
    void* from_, void* to_, npy_intp n,
    void* /*fromarr*/, void* /*toarr*/) {
    const From* from_inner = reinterpret_cast<From*>(from_);
    To* to_inner = reinterpret_cast<To*>(to_);
    for (npy_intp i = 0; i < n; i++)
      to_inner[i] = cast_lambda(from_inner[i]);
  };
  auto& api = npy_api::get();
  if (!from) {
  from = npy_format_descriptor<From>::dtype();
  }
  if (!to) {
  to = npy_format_descriptor<To>::dtype();
  }
  int to_num = to.num();
  auto from_raw = reinterpret_cast<PyArray_Descr*>(from.ptr());
  if (from.num() == npy_api::NPY_OBJECT_ && !std::is_same<From, object>::value)
    pybind11_fail(
      "ufunc: Registering conversion from `dtype=object` with "
      "From != `py::object` is not supported");
  if (api.PyArray_RegisterCastFunc_(from_raw, to_num, cast_func) < 0) {
    pybind11_fail("ufunc: Cannot register cast");
  }
  if (allow_coercion) {
  if (api.PyArray_RegisterCanCast_(
      from_raw, to_num, npy_api::NPY_NOSCALAR_) < 0)
    pybind11_fail(
      "ufunc: Cannot register implicit / coercion cast capability");
  }
}

}  // namespace detail

/**
Defines a UFunc in NumPy.
Handles either 1 or 2 arguments (for unary or binary arguments).
@pre All classes used must have corresponding dtypes in NumPy.
@note You must specify the class that is going to own the UFunc, due to
NumPy's API design.

Example:

    py::ufunc(m, "custom_ufunc")
        .def_loop<CustomClass>([](const CustomClass& a) { ... });
 */
class ufunc : public object {
 public:
  /// Enables defining a new UFunc `name`  in `scope`.
  ufunc(handle scope, const char* name) : scope_{scope} {
    entries.reset(new entries_t(name));
  }

  // Wraps a reference to an existing UFunc.
  // NOLINTNEXTLINE(runtime/explicit)
  ufunc(object ptr_in) : object(ptr_in) {
    // TODO(eric.cousineau): Check type.
    if (!self() || self().is_none()) {
      pybind11_fail("ufunc: Cannot wrap from empty or None object");
    }
    entries.reset(new entries_t(ptr()));
  }

  /// Constructs from a raw pointer.
  explicit ufunc(detail::PyUFuncObject* ptr_in)
    : ufunc(reinterpret_borrow<object>(reinterpret_cast<PyObject*>(ptr_in)))
  {}

  ufunc(const ufunc&) = default;

  /// "Flushes" queued UFunc definitions.
  ~ufunc() {
    if (entries) {
      finalize();
    }
  }

  /// Gets a NumPy builtin UFunc by name.
  static ufunc get_builtin(const char* name) {
    module numpy = module::import("numpy");
    return ufunc(numpy.attr(name));
  }

  /// Queues a function to be realized as a UFunc loop.
  template <typename Type, typename Func>
  ufunc& def_loop(Func func_in) {
    auto func = detail::infer_function_info(func_in).func;
    do_register<Type>(detail::ufunc_to_ptr(func));
    return *this;
  }

  /// Retrieves raw pointer.
  detail::PyUFuncObject* ptr() const {
    return reinterpret_cast<detail::PyUFuncObject*>(self().ptr());
  }

 private:
  object& self() { return *this; }
  const object& self() const { return *this; }

  // Create UFunc object with core type functions if needed, and register user
  // functions.
  void finalize() {
    if (!entries)
      pybind11_fail("Object already finalized");
    if (!self()) {
      // Create object and register core functions.
      auto* h = entries->create_core();
      self() = reinterpret_borrow<object>(reinterpret_cast<PyObject*>(h));
      scope_.attr(entries->name()) = self();
    }
    // Register user type functions.
    entries->create_user(ptr());
    // Leak memory for now so that data lives longer than UFunc.
    // TODO(eric.cousineau): Embed this in a capsule and use `keep_alive`.
    (void)new std::shared_ptr<entries_t>(entries);
  }

  // Registers a function pointer as a UFunc, mapping types to dtype nums.
  template <typename Type, typename ... Args>
  void do_register(detail::ufunc_ptr<Args...> user) {
    constexpr int N = sizeof...(Args);
    constexpr int nin = N - 1;
    constexpr int nout = 1;
    entries->init_or_check_args(nin, nout);

    const int dtype = dtype::of<Type>().num();
    const int dummy[] = {(detail::assert_ufunc_dtype_valid<Args>(), 0)...};
    (void)dummy;
    const std::vector<int> dtype_args = {dtype::of<Args>().num()...};
    bool is_core = true;
    for (int i = 0; i < N; ++i) {
      const size_t ii = static_cast<size_t>(i);
      if (dtype_args[ii] >= detail::npy_api::constants::NPY_USERDEF_)
        is_core = false;
    }
    if (is_core) {
      // TODO(eric.cousineau): Consider supporting
      // `PyUFunc_ReplaceLoopBySignature_`?
      if (self())
        pybind11_fail(
            "ufunc: Can't add/replace signatures for core types for an "
            "existing ufunc");
      entries->queue_core(user.func, user.data, dtype_args);
    } else {
      entries->queue_user(user.func, user.data, dtype, dtype_args);
    }
  }

  // These are only used if we have something new.
  handle scope_{};

  // Contains UFunc entries to flush into actual UFunc registrations.
  class entries_t {
   public:
    // Initialize from existing object.
    explicit entries_t(detail::PyUFuncObject* h) {
      nin_ = h->nin;
      nout_ = h->nout;
    }

    // Set up to create a new instance.
    explicit entries_t(const char* name) : name_(name) {}

    void init_or_check_args(int nin, int nout) {
      if (nin_ != -1 && nout_ != -1) {
        if (nin_ != nin)
          pybind11_fail("ufunc: Input count mismatch");
        if (nout_ != nout)
          pybind11_fail("ufunc: Output count mismatch");
      }
      nin_ = nin;
      nout_ = nout;
    }

    const char* name() const { return name_.c_str(); }

    void queue_core(
        detail::PyUFuncGenericFunction func, void* data,
        const std::vector<int>& dtype_args) {
      assert(nin_ != -1 && nout_ != -1);
      assert(static_cast<int>(dtype_args.size()) == nin_ + nout_);
      // Store core functionn.
      core_funcs_.push_back(func);
      core_data_.push_back(data);
      const size_t ncore = core_funcs_.size();
      size_t t_index = core_type_args_.size();
      int nargs = nin_ + nout_;
      core_type_args_.resize(ncore * static_cast<size_t>(nargs));
      for (size_t i = 0; i < dtype_args.size(); ++i) {
        core_type_args_.at(t_index++) = static_cast<char>(dtype_args[i]);
      }
    }

    void queue_user(
        detail::PyUFuncGenericFunction func, void* data, int dtype,
        const std::vector<int>& dtype_args) {
      assert(nin_ != -1 && nout_ != -1);
      assert(static_cast<int>(dtype_args.size()) == nin_ + nout_);
      user_funcs_.push_back(func);
      user_data_.push_back(data);
      user_types_.push_back(dtype);
      user_type_args_.push_back(dtype_args);
    }

    detail::PyUFuncObject* create_core() {
      const int ncore = static_cast<int>(core_funcs_.size());
      char* name_raw = const_cast<char*>(name());
      return reinterpret_cast<detail::PyUFuncObject*>(
          detail::npy_api::get().PyUFunc_FromFuncAndData_(
            core_funcs_.data(), core_data_.data(), core_type_args_.data(),
            ncore, nin_, nout_,
            detail::npy_api::constants::PyUFunc_None_, name_raw, nullptr, 0));
    }

    void create_user(detail::PyUFuncObject* h) {
      const size_t nuser = user_funcs_.size();
      for (size_t i = 0; i < nuser; ++i) {
        if (detail::npy_api::get().PyUFunc_RegisterLoopForType_(
                h, user_types_[i], user_funcs_[i],
                user_type_args_[i].data(), user_data_[i]) < 0)
          pybind11_fail("ufunc: Failed to register custom ufunc");
      }
    }

   private:
    int nin_{-1};
    int nout_{-1};
    std::string name_{};

    // Core.
    std::vector<detail::PyUFuncGenericFunction> core_funcs_;
    std::vector<void*> core_data_;
    std::vector<char> core_type_args_;

    // User.
    std::vector<detail::PyUFuncGenericFunction> user_funcs_;
    std::vector<void*> user_data_;
    std::vector<int> user_types_;
    std::vector<std::vector<int>> user_type_args_;
  };

  std::shared_ptr<entries_t> entries;
};

}  // namespace pybind11

#pragma GCC visibility pop
