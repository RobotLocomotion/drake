#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace drake {

namespace detail {

// Collects both a functor object and its signature for ease of inference.
template <typename Func, typename Return, typename ... Args>
struct function_info {
  // TODO(eric.cousineau): Ensure that this permits copy elision when combined
  // with `std::forward<Func>(func)`, while still behaving well with primitive
  // types.
  std::decay_t<Func> func;
};

// Implementation for inferring `function_info<>` from a generic `Func` object.
template <typename Return, typename ... Args, typename Func>
auto infer_function_info_impl(Func&& func, Return (*infer)(Args...) = nullptr) {
  (void)infer;
  return function_info<Func, Return, Args...>{std::forward<Func>(func)};
}

// SFINAE for functors.
// N.B. This *only* distinguished between function / method pointers and
// lambda objects. It does *not* distinguish among other types.
template <typename Func, typename T = void>
using enable_if_lambda_t =
    std::enable_if_t<std::integral_constant<
        bool, !std::is_function<std::decay_t<Func>>::value>::value, T>;

// Infers `function_info<>` from a function pointer.
template <typename Return, typename ... Args>
auto infer_function_info(Return (*func)(Args...)) {
  return infer_function_info_impl<Return, Args...>(func);
}

// Infers `function_info<>` from a mutable method pointer.
template <typename Return, typename Class, typename ... Args>
auto infer_function_info(Return (Class::*method)(Args...)) {
  auto func = [method](Class* self, Args... args) {
    return (self->*method)(std::forward<Args>(args)...);
  };
  return infer_function_info_impl<Return, Class*, Args...>(func);
}

// Infers `function_info<>` from a const method pointer.
template <typename Return, typename Class, typename ... Args>
auto infer_function_info(Return (Class::*method)(Args...) const) {
  auto func = [method](const Class* self, Args... args) {
    return (self->*method)(std::forward<Args>(args)...);
  };
  return infer_function_info_impl<Return, const Class*, Args...>(func);
}

// Helpers for general functor objects.
struct functor_helpers {
  // Removes class from mutable method pointer for inferring signature
  // of functor.
  template <typename Class, typename Return, typename ... Args>
  static auto remove_class_from_ptr(Return (Class::*)(Args...)) {
    return (Return (*)(Args...)){};  // NOLINT
  }

  // Removes class from const method pointer for inferring signature of functor.
  template <typename Class, typename Return, typename ... Args>
  static auto remove_class_from_ptr(Return (Class::*)(Args...) const) {
    return (Return (*)(Args...)){};  // NOLINT
  }

  // Infers funtion pointer from functor.
  // @pre `Func` must have only *one* overload of `operator()`.
  template <typename Func>
  static auto infer_function_ptr() {
    return remove_class_from_ptr(&Func::operator());
  }
};

// Infers `function_info<>` from a generic functor.
template <typename Func, typename = detail::enable_if_lambda_t<Func>>
auto infer_function_info(Func&& func) {
  return infer_function_info_impl(
      std::forward<Func>(func),
      functor_helpers::infer_function_ptr<std::decay_t<Func>>());
}

// Implementation for wrapping a function by scanning and replacing arguments
// based on their types.
template <template <typename...> class wrap_arg_tpl>
struct wrap_function_impl {
  // Customization of input template to specialize for arguments of the form
  // `std::function<>`.
  template <typename T>
  struct wrap_arg : public wrap_arg_tpl<T> {};

  // Provide specialized type extractor so that we can catch `T = void`.
  // Use `Extra` so that we can specialize within class scope.
  template <typename T, typename Extra>
  struct wrap_arg_t_impl {
    using type = decltype(wrap_arg<T>::wrap(std::declval<T>()));
  };

  // Intercept `void`, since `declval<void>()` is invalid.
  template <typename Extra>
  struct wrap_arg_t_impl<void, Extra> {
    using type = void;
  };

  // Convenience wrapper.
  template <typename T>
  using wrap_arg_t = typename wrap_arg_t_impl<T, void>::type;

  // Determines which overload should be used, since we cannot wrap a `void`
  // type using `wrap_arg<void>::wrap()`.
  template <typename Return>
  static constexpr bool enable_wrap_output =
      !std::is_same<Return, void>::value;

  // Wraps function arguments and the return value.
  // Generally used when `Return` is non-void.
  template <typename Func, typename Return, typename ... Args>
  static auto run(function_info<Func, Return, Args...>&& info,
      std::enable_if_t<enable_wrap_output<Return>, void*> = {}) {
    // N.B. Since we do not use the `mutable` keyword with this lambda,
    // any functors passed in *must* have provide `operator()(...) const`.
    auto func_wrapped =
        [func_f = std::forward<Func>(info.func)]
        (wrap_arg_t<Args>... args_wrapped) -> wrap_arg_t<Return> {
      return wrap_arg<Return>::wrap(
          func_f(wrap_arg<Args>::unwrap(
                  std::forward<wrap_arg_t<Args>>(args_wrapped))...));
    };
    return func_wrapped;
  }

  // Wraps function arguments, but not the return value.
  // Generally used when `Return` is void.
  template <typename Func, typename Return, typename ... Args>
  static auto run(function_info<Func, Return, Args...>&& info,
      std::enable_if_t<!enable_wrap_output<Return>, void*> = {}) {
    auto func_wrapped =
        [func_f = std::forward<Func>(info.func)]
        (wrap_arg_t<Args>... args_wrapped) -> Return {
      return func_f(wrap_arg<Args>::unwrap(
              std::forward<wrap_arg_t<Args>>(args_wrapped))...);
    };
    return func_wrapped;
  }

  // Specialization for callbacks of the form `std::function<>`.
  // @note We could generalize this using SFINAE for functors of any form, but
  // that complicates the details for a relatively low ROI.
  template <typename Return, typename ... Args>
  struct wrap_arg<const std::function<Return (Args...)>&> {  // NOLINT
    // Define types explicit, since `auto` is not easily usable as a return type
    // (compilers struggle with inference).
    using Func = std::function<Return (Args...)>;
    using WrappedFunc = std::function<wrap_arg_t<Return> (wrap_arg_t<Args>...)>;

    static WrappedFunc wrap(const Func& func) {
      return wrap_function_impl::run(infer_function_info(func));
    }

    // Unwraps a `WrappedFunc`, also unwrapping the return value.
    // @note We use `Defer` so that we can use SFINAE without a disptach method.
    template <typename Defer = Return>
    static Func unwrap(
        const WrappedFunc& func_wrapped,
        std::enable_if_t<enable_wrap_output<Defer>, void*> = {}) {
      return [func_wrapped](Args... args) -> Return {
        return wrap_arg<Return>::unwrap(
            func_wrapped(wrap_arg<Args>::wrap(std::forward<Args>(args))...));
      };
    }

    // Specialization / overload of above, but not wrapping the return value.
    template <typename Defer = Return>
    static Func unwrap(
        const WrappedFunc& func_wrapped,
        std::enable_if_t<!enable_wrap_output<Defer>, void*> = {}) {
      return [func_wrapped](Args... args) {
        func_wrapped(wrap_arg<Args>::wrap(std::forward<Args>(args))...);
      };
    }
  };

  // Ensure that we also wrap `std::function<>` returned by value.
  template <typename Signature>
  struct wrap_arg<std::function<Signature>>
      : public wrap_arg<const std::function<Signature>&> {};
};

}  // namespace detail

/// Default case for argument wrapping, with pure pass-through. Consider
/// inheriting from this for base cases.
/// N.B. `Wrapped` is not necessary, but is used for demonstration purposes.
template <typename T>
struct wrap_arg_default {
  using Wrapped = T;
  static Wrapped wrap(T arg) { return std::forward<T&&>(arg); }
  static T unwrap(Wrapped arg_wrapped) {
    return std::forward<Wrapped&&>(arg_wrapped);
  }
  // N.B. `T` rather than `T&&` is used as arguments here as it behaves well
  // with primitve types, such as `int`.
};

/// Wraps a functions arguments and its return value (if non-void) based on
/// `wrap_arg_tpl`. This will also recursively handle wrapping /unwrapping
/// `std::function<>` arguments and return values.
/// @tparam wrap_arg_tpl
///   User-supplied argument wrapper, that must supply
///   `wrap_arg_tpl<T>::wrap(auto arg, ...)` and
///   `wrap_arg_tpl<T>::unwrap(auto arg_wrapped, ...)`. See `wrap_arg_default`
///   for a simple implementation.
///   N.B. This template template parameter uses a parameter pack to allow
///   for SFINAE. If passing a `using` template alias, ensure that the alias
///   template template parameter uses a parameter pack of the *exact* same
///   form.
/// @param func
///   Functor to be wrapped. Returns a function with wrapped arugments and
///   return type. If functor is a method pointer, it will return a function of
///   the form `Return ([const] Class* self, ...)`.
/// @return Wrapped function lambda.
///   N.B. Construct a `std::function<>` from this if you encounter inference
///   issues downstream of this method.
template <template <typename...> class wrap_arg_tpl, typename Func>
auto WrapFunction(Func&& func) {
  // TODO(eric.cousineau): Create an overload with `type_pack<Args...>` to
  // handle overloads, to disambiguate when necessary.
  return detail::wrap_function_impl<wrap_arg_tpl>::run(
      detail::infer_function_info(std::forward<Func>(func)));
}

}  // namespace drake
