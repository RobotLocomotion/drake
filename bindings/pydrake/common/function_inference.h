#pragma once

#include <type_traits>
#include <utility>

#include "drake/bindings/pydrake/common/type_pack.h"

// TODO(eric.cousineau): Figure out how to make this automatically hidden.
#pragma GCC visibility push(hidden)

namespace drake {
namespace pydrake {
namespace detail {

struct function_inference {
  // Collects both a functor object and its signature for ease of inference.
  template <typename Func_, typename ReturnT, typename ... ArgsT>
  struct info {
    // TODO(eric.cousineau): Ensure that this permits copy elision when combined
    // with `std::forward<Func>(func)`, while still behaving well with primitive
    // types.
    using Return = ReturnT;
    using Args = type_pack<ArgsT...>;
    using Func = std::decay_t<Func_>;
    Func func;
  };

  // Factory method for `info<>`, to be used by `run`.
  template <typename Return, typename ... Args, typename Func>
  static auto make_inferred_info(
      Func&& func, Return (*infer)(Args...) = nullptr) {
    (void)infer;
    return info<Func, Return, Args...>{std::forward<Func>(func)};
  }

  // Infers `info<>` from a function pointer.
  template <typename Return, typename ... Args>
  static auto run(Return (*func)(Args...)) {
    return make_inferred_info<Return, Args...>(func);
  }

  // Infers `info<>` from a mutable method pointer.
  template <typename Return, typename Class, typename ... Args>
  static auto run(Return (Class::*method)(Args...)) {
    auto func = [method](Class& self, Args... args) {
      return (self.*method)(std::forward<Args>(args)...);
    };
    return make_inferred_info<Return, Class&, Args...>(func);
  }

  // Infers `info<>` from a const method pointer.
  template <typename Return, typename Class, typename ... Args>
  static auto run(Return (Class::*method)(Args...) const) {
    auto func = [method](const Class& self, Args... args) {
      return (self.*method)(std::forward<Args>(args)...);
    };
    return make_inferred_info<Return, const Class&, Args...>(func);
  }

  // Helpers for general functor objects.
  struct infer_helper {
    // Removes class from mutable method pointer for inferring signature
    // of functor.
    template <typename Class, typename Return, typename ... Args>
    static auto remove_class_from_ptr(Return (Class::*)(Args...)) {
      using Ptr = Return (*)(Args...);
      return Ptr{};
    }

    // Removes class from const method pointer for inferring signature of
    // functor.
    template <typename Class, typename Return, typename ... Args>
    static auto remove_class_from_ptr(Return (Class::*)(Args...) const) {
      using Ptr = Return (*)(Args...);
      return Ptr{};
    }

    // Infers funtion pointer from functor.
    // @pre `Func` must have only *one* overload of `operator()`.
    template <typename Func>
    static auto infer_function_ptr() {
      return remove_class_from_ptr(&Func::operator());
    }
  };

  // SFINAE for functors.
  // N.B. This *only* distinguished between function / method pointers and
  // lambda objects. It does *not* distinguish among other types.
  template <typename Func, typename T = void>
  using enable_if_lambda_t =
      std::enable_if_t<!std::is_function<std::decay_t<Func>>::value, T>;

  // Infers `info<>` from a generic functor.
  template <typename Func, typename = enable_if_lambda_t<Func>>
  static auto run(Func&& func) {
    return make_inferred_info(
        std::forward<Func>(func),
        infer_helper::infer_function_ptr<std::decay_t<Func>>());
  }
};

// Infers `function_inference::info<>` from a generic functor, converting form
// the left form to the right form:
//   <lambda>  ->  Return(*)(Args...)
//   Return (Class::*)(Args...)  ->  Return (*)(Class&, Args...)
//   Return (Class::*)(Args...) const  ->  Return (*)(const Class&, Args...)
template <typename Func>
auto infer_function_info(Func&& func) {
  return function_inference::run(std::forward<Func>(func));
}

}  // namespace detail
}  // namespace pydrake
}  // namespace drake

#pragma GCC visibility pop
