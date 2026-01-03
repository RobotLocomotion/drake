#pragma once

#include <string>
#include <string_view>
#include <type_traits>

#include <fmt/format.h>

// This file contains the essentials of fmt support in Drake, mainly
// compatibility code to inter-operate with different versions of fmt.

// N.B. The spelling of the macro names between doc/Doxyfile_CXX.in and this
// file should be kept in sync.

namespace drake {

#if FMT_VERSION >= 80000 || defined(DRAKE_DOXYGEN_CXX)
/** When using fmt >= 8, this is an alias for
<a
href="https://fmt.dev/latest/api.html#compile-time-format-string-checks">fmt::runtime</a>.
When using fmt < 8, this is a no-op. */
inline auto fmt_runtime(std::string_view s) {
  return fmt::runtime(s);
}
/** When using fmt >= 8, this is defined to be `const` to indicate that the
`fmt::formatter<T>::format(...)` function should be object-const.
When using fmt < 8, the function signature was incorrect (lacking the const),
so this macro will be empty. */
#define DRAKE_FMT8_CONST const
#else  // FMT_VERSION
inline auto fmt_runtime(std::string_view s) {
  return s;
}
#define DRAKE_FMT8_CONST
#endif  // FMT_VERSION

/** Returns `fmt::to_string(x)` but always with at least one digit after the
decimal point. Different versions of fmt disagree on whether to omit the
trailing ".0" when formatting integer-valued floating-point numbers.
@tparam T must be either `float` or `double`. */
template <typename T>
std::string fmt_floating_point(T x)
  requires(std::is_same_v<T, float> || std::is_same_v<T, double>);

template <typename T>
[[deprecated(
    "\nDRAKE DEPRECATED: The fmt_floating_point function now only allows "
    "'float' and 'double' as template arguments; other types are deprecated.\n"
    "The deprecated code will be removed from Drake on or after 2026-02-01.")]]
std::string fmt_floating_point(T x)
  requires(!(std::is_same_v<T, float> || std::is_same_v<T, double>))
{
  std::string result = fmt::format("{:#}", x);
  if (result.back() == '.') {
    result.push_back('0');
  }
  return result;
}

/** Returns `fmt::("{:?}", x)`, i.e, using fmt's "debug string format"; see
https://fmt.dev docs for the '?' presentation type for details. We provide this
wrapper because not all of our supported platforms have a new-enough fmt
to rely on it. On platforms with older fmt, we use a Drake re-implementation
of the feature that does NOT handle unicode correctly. */
std::string fmt_debug_string(std::string_view x);

namespace internal::formatter_as {

/* The DRAKE_FORMATTER_AS macro specializes this for it's format_as types.
This struct is a functor that performs a conversion. See below for details.
@tparam T is the TYPE to be formatted. */
template <typename T>
struct Converter;

/* Provides convenient type shortcuts for the TYPE in DRAKE_FORMATTER_AS. */
template <typename T>
struct Traits {
  /* The type being formatted. */
  using InputType = T;
  /* The format_as functor that provides the alternative object to format. */
  using Functor = Converter<T>;
  /* The type of the alternative object. */
  using OutputType = decltype(Functor::call(std::declval<InputType>()));
  /* The fmt::formatter<> for the alternative object. */
  using OutputTypeFormatter = fmt::formatter<OutputType>;
};

/* Implements fmt::formatter<TYPE> for DRAKE_FORMATER_AS types. The macro
specializes this for it's format_as types.  See below for details.
@tparam T is the TYPE to be formatted. */
template <typename T>
struct Formatter;

}  // namespace internal::formatter_as

}  // namespace drake

/** Adds a `fmt::formatter<NAMESPACE::TYPE>` template specialization that
formats the `TYPE` by delegating the formatting using a transformed expression,
as if a conversion function like this were interposed during formatting:

@code{cpp}
template <TEMPLATE_ARGS>
auto format_as(const NAMESPACE::TYPE& ARG) {
  return EXPR;
}
@endcode

For example, this declaration ...

@code{cpp}
DRAKE_FORMATTER_AS(, my_namespace, MyType, x, x.to_string())
@endcode

... changes this code ...

@code{cpp}
MyType foo;
fmt::print(foo);
@endcode

... to be equivalent to ...

@code{cpp}
MyType foo;
fmt::print(foo.to_string());
@endcode

... allowing user to format `my_namespace::MyType` objects without manually
adding the `to_string` call every time.

This provides a convenient mechanism to add formatters for classes that already
have a `to_string` function, or classes that are just thin wrappers over simple
types (ints, enums, etc.).

Always use this macro in the global namespace, and do not use a semicolon (`;`)
afterward.

@param TEMPLATE_ARGS The optional first argument `TEMPLATE_ARGS` can be used in
case the `TYPE` is templated, e.g., it might commonly be set to `typename T`. It
should be left empty when the TYPE is not templated. In case `TYPE` has multiple
template arguments, note that macros will fight with commas so you should use
`typename... Ts` instead of writing them all out.

@param NAMESPACE The namespace that encloses the `TYPE` being formatted. Cannot
be empty. For nested namespaces, use intermediate colons, e.g.,
`%drake::common`. Do not place _leading_ colons on the `NAMESPACE`.

@param TYPE The class name (or struct name, or enum name, etc.) being formatted.
Do not place _leading_ double-colons on the `TYPE`. If the type is templated,
use the template arguments here, e.g., `MyOptional<T>` if `TEMPLATE_ARGS` was
chosen as `typename T`.

@param ARG A placeholder variable name to use for the value (i.e., object)
being formatted within the `EXPR` expression.

@param EXPR An expression to `return` from the format_as function; it can
refer to the given `ARG` name which will be of type `const TYPE& ARG`. The
evaluated expression can only ever throw exceptions that `std::string`'s
default allocator might throw (i.e., `std::bad_alloc`).

@note In future versions of fmt (perhaps fmt >= 10) there might be an ADL
`format_as` customization point with this feature built-in. If so, then we can
update this macro to use that spelling, and eventually deprecate the macro once
Drake drops support for earlier version of fmt. */
#define DRAKE_FORMATTER_AS(TEMPLATE_ARGS, NAMESPACE, TYPE, ARG, EXPR)          \
  /* Specializes the Converter<> class template for our TYPE. */               \
  namespace drake::internal::formatter_as {                                    \
  template <TEMPLATE_ARGS>                                                     \
  struct Converter<NAMESPACE::TYPE> {                                          \
    using InputType = NAMESPACE::TYPE;                                         \
    static auto call(const InputType& ARG) {                                   \
      return EXPR;                                                             \
    }                                                                          \
  };                                                                           \
                                                                               \
  /* Provides the fmt::formatter<TYPE> implementation. */                      \
  template <TEMPLATE_ARGS>                                                     \
  struct Formatter<NAMESPACE::TYPE>                                            \
      : fmt::formatter<typename Traits<NAMESPACE::TYPE>::OutputType> {         \
    using MyTraits = Traits<NAMESPACE::TYPE>;                                  \
    /* Shadow our base class member function template of the same name. */     \
    template <typename FormatContext>                                          \
    auto format(const typename MyTraits::InputType& x,                         \
                FormatContext& ctx) DRAKE_FMT8_CONST {                         \
      /* Call the base class member function after laundering the object   */  \
      /* through the user's provided format_as function. Older versions of */  \
      /* fmt have const-correctness bugs, which we can fix with some good  */  \
      /* old fashioned const_cast-ing here.                                */  \
      using Base = typename MyTraits::OutputTypeFormatter;                     \
      const Base* const self = this;                                           \
      return const_cast<Base*>(self)->format(MyTraits::Functor::call(x), ctx); \
    }                                                                          \
  };                                                                           \
  } /* namespace drake::internal::formatter_as */                              \
                                                                               \
  /* Specializes the fmt::formatter<> class template for TYPE. */              \
  namespace fmt {                                                              \
  template <TEMPLATE_ARGS>                                                     \
  struct formatter<NAMESPACE::TYPE>                                            \
      : drake::internal::formatter_as::Formatter<NAMESPACE::TYPE> {};          \
  } /* namespace fmt */
