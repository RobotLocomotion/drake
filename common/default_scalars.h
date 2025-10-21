#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/symbolic/expression.h"

// N.B. `CommonScalarPack` and `NonSymbolicScalarPack` in `systems_pybind.h`
// should be kept in sync with this file.

// N.B. The spelling of the macro names between doc/Doxyfile_CXX.in and this
// file should be kept in sync.

/// @defgroup default_scalars Default Scalars
/// @ingroup technical_notes
/// @{
/// Similar to the Eigen library, many classes in Drake use a template argument
/// to specify the numeric scalar type to use for computation.  We typically
/// name that template argument <b>`<T>`</b>.  For an example, see the class
/// drake::math::RigidTransform.
///
/// Most scalar-templated classes in Drake only support a small, fixed set of
/// scalar types:
/// - `double` (always)
/// - drake::AutoDiffXd (almost always)
/// - drake::symbolic::Expression (sometimes)
///
/// When Drake documentation refers to "default scalars", it means all three
/// of the types above.
///
/// Alternatively, reference to "default nonsymbolic scalars" means all except
/// `drake::symbolic::Expression`.
///
/// For code within Drake, we offer Doxygen custom commands to document the
/// `<T>` template parameter in the conventional cases:
///
/// - @c \@tparam_default_scalar for the default scalars.
/// - @c \@tparam_nonsymbolic_scalar for the default nonsymbolic scalars.
/// - @c \@tparam_double_only for only the `double` scalar and nothing else.
///
/// All three commands assume that the template parameter is named `T`.  When
/// possible, prefer to use these commands instead of writing out a @c \@tparam
/// line manually.

/// @name Class template instantiation macros
/// These macros either declare or define class template instantiations for
/// Drake's supported scalar types (see @ref default_scalars), either "default
/// scalars" or "default nonsymbolic scalars".  Use the `DECLARE` macros only
/// in .h files; use the `DEFINE` macros only in .cc files.
///
/// @param SomeType the template typename to instantiate, *including* the
/// leading `class` or `struct` keyword, but *excluding* the template argument
/// for the scalar type.
///
/// Example `my_system.h`:
/// @code
/// #include "drake/common/default_scalars.h"
///
/// namespace sample {
/// template <typename T>
/// class MySystem final : public drake::systems::LeafSystem<T> {
///   ...
/// };
/// }  // namespace sample
///
/// DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
///     class ::sample::MySystem);
/// @endcode
///
/// Example `my_system.cc`:
/// @code
/// #include "my_system.h"
///
/// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
///     class ::sample::MySystem);
/// @endcode
///
/// See also @ref system_scalar_conversion.
/// @{

namespace drake {
// A constant that is true iff T is a default scalar type.
template <typename T>
constexpr bool is_default_scalar =
    std::is_same_v<T, double> || std::is_same_v<T, drake::AutoDiffXd> ||
    std::is_same_v<T, drake::symbolic::Expression>;
}  // namespace drake

// In support of the macros below, we define a struct template that provides
// the per-phase typedef `scalar_phase_t<DRAKE_ONCE_PER_SCALAR_PHASE>::type`.
// If more scalar types are added, also fix drake/tools/skylark/drake_cc.bzl
// to reflect the new count of default scalars.
namespace drake {
namespace internal {
template <int>
struct scalar_phase_t {};
template <>
struct scalar_phase_t<0> {
  using type = double;
};
template <>
struct scalar_phase_t<1> {
  using type = ::drake::AutoDiffXd;
};
template <>
struct scalar_phase_t<2> {
  using type = ::drake::symbolic::Expression;
};
}  // namespace internal
}  // namespace drake

// This preprocessor code doesn't auto-format very well.
// clang-format off
#ifndef DRAKE_ONCE_PER_SCALAR_PHASE
/// Defines template instantiations for Drake's default scalars.
/// This should only be used in .cc files, never in .h files.
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>; \
template SomeType<::drake::symbolic::Expression>;
#else
#define DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    SomeType) \
template SomeType< \
    ::drake::internal::scalar_phase_t<DRAKE_ONCE_PER_SCALAR_PHASE>::type>;
#endif

#ifndef DRAKE_ONCE_PER_SCALAR_PHASE
/// Defines template instantiations for Drake's default nonsymbolic scalars.
/// This should only be used in .cc files, never in .h files.
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
template SomeType<double>; \
template SomeType<::drake::AutoDiffXd>;
#elif DRAKE_ONCE_PER_SCALAR_PHASE <= 1  /* Skip phase 2 (Expression). */
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
template SomeType< \
    ::drake::internal::scalar_phase_t<DRAKE_ONCE_PER_SCALAR_PHASE>::type>;
#else
#define \
  DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
static_assert(true);
#endif

/// Declares that template instantiations exist for Drake's default scalars.
/// This should only be used in .h files, never in .cc files.
#define DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(  \
    SomeType) \
extern template SomeType<double>; \
extern template SomeType<::drake::AutoDiffXd>; \
extern template SomeType<::drake::symbolic::Expression>;

/// Declares that template instantiations exist for Drake's default nonsymbolic
/// scalars.  This should only be used in .h files, never in .cc files.
#define \
  DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
      SomeType) \
extern template SomeType<double>; \
extern template SomeType<::drake::AutoDiffXd>;
// clang-format on

/// @}

/// @name Function template instantiation macros
/// These macros define template function instantiations for Drake's supported
/// scalar types (see @ref default_scalars), either "default scalars" or
/// "default nonsymbolic scalars".  Use the `DEFINE` macros only in .cc files.
///
/// @param FunctionPointersTuple a parenthesized, comma-separated list of
/// functions to instantiate (provided as function pointers), *including*
/// the template argument(s) for the scalar type.  The template arguments `T`
/// and `U` will be provided by the macro; the function will be instantiated
/// for all combinations of `T` and `U` over the default scalars.
///
/// Example `example.h`:
/// @code
/// #include "drake/common/default_scalars.h"
///
/// namespace sample {
///
/// template <typename T>
/// double Func1(const T&);
///
/// template <typename T, typename U>
/// double Func2(const T&, const U&);
///
/// template <typename T>
/// class SomeClass {
///   ...
///   template <typename U>
///   SomeClass cast() const;
///   ...
/// };
///
/// }  // namespace sample
/// @endcode
///
/// Example `example.cc`:
/// @code
/// #include "example.h"
///
/// namespace sample {
///
/// template <typename T>
/// double Func1(const T&) {
///   ...
/// }
///
/// template <typename T, typename U>
/// double Func2(const T&, const U&) {
///   ...
/// }
///
/// template <typename T>
/// template <typename U>
/// SomeClass<T>::SomeClass::cast() const {
///   ...
/// };
///
/// // N.B. Place the macro invocation inside the functions' namespace.
/// DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
///     &Func1<T>,
///     &Func2<T, U>,
///     &SomeClass<T>::template cast<U>
/// ));
///
/// }  // namespace sample
/// @endcode
///
/// @note In the case of an overloaded function, the `&FunctionName<T>` syntax
/// is ambiguous.  To resolve the ambiguity, you will need a
/// <a
/// href=https://en.cppreference.com/w/cpp/language/static_cast#Notes>static_cast</a>.

// N.B. Below we use "Make_Function_Pointers" (etc.) as function names and
// static variable names, which violates our function name style guide by mixing
// inner capitalization with underscores.  However, we've done this on purpose,
// to avoid conflicts with user-provided code in the same translation unit.  We
// can't use a namespace because we need to allow for easy friendship in case a
// member function does not have public access.

// This preprocessor code doesn't auto-format very well.
// clang-format off
#ifndef DRAKE_ONCE_PER_SCALAR_PHASE
/// Defines template instantiations for Drake's default scalars.
/// This should only be used in .cc files, never in .h files.
#define DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    FunctionPointersTuple) \
template<typename T, typename U> \
constexpr auto Make_Function_Pointers() { \
  return std::make_tuple FunctionPointersTuple ; \
} \
template<typename T, typename... Us> \
constexpr auto Make_Function_Pointers_Pack2() { \
  return std::tuple_cat(Make_Function_Pointers<T, Us>()...); \
} \
template<typename... Ts> \
constexpr auto Make_Function_Pointers_Pack1() { \
  return std::tuple_cat(Make_Function_Pointers_Pack2<Ts, Ts...>()...); \
} \
static constexpr auto Function_Templates __attribute__((used)) = \
    Make_Function_Pointers_Pack1< \
        double, \
        ::drake::AutoDiffXd, \
        ::drake::symbolic::Expression>();
#else
#define DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS( \
    FunctionPointersTuple) \
template<typename T, typename U> \
constexpr auto Make_Function_Pointers() { \
  return std::make_tuple FunctionPointersTuple ; \
} \
template<typename T, typename... Us> \
constexpr auto Make_Function_Pointers_Pack2() { \
  return std::tuple_cat(Make_Function_Pointers<T, Us>()...); \
} \
static constexpr auto Function_Templates __attribute__((used)) = \
    Make_Function_Pointers_Pack2< \
        ::drake::internal::scalar_phase_t<DRAKE_ONCE_PER_SCALAR_PHASE>::type, \
        double, \
        ::drake::AutoDiffXd, \
        ::drake::symbolic::Expression>();  /* NOLINT(whitespace/operators) */
#endif

#ifndef DRAKE_ONCE_PER_SCALAR_PHASE
/// Defines template instantiations for Drake's default nonsymbolic scalars.
/// This should only be used in .cc files, never in .h files.
#define \
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
    FunctionPointersTuple) \
template<typename T, typename U> \
constexpr auto Make_Function_Pointers_Nonsym() { \
  return std::make_tuple FunctionPointersTuple ; \
} \
template<typename T, typename... Us> \
constexpr auto Make_Function_Pointers_Nonsym_Pack2() { \
  return std::tuple_cat(Make_Function_Pointers_Nonsym<T, Us>()...); \
} \
template<typename... Ts> \
constexpr auto Make_Function_Pointers_Nonsym_Pack1() { \
  return std::tuple_cat(Make_Function_Pointers_Nonsym_Pack2<Ts, Ts...>()...); \
} \
/* NOLINTNEXTLINE(readability/fn_size) */ \
static constexpr auto Function_Templates_Nonsym __attribute__((used)) = \
    Make_Function_Pointers_Nonsym_Pack1< \
        double, \
        ::drake::AutoDiffXd>();
#elif DRAKE_ONCE_PER_SCALAR_PHASE <= 1  /* Skip phase 2 (Expression). */
#define \
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
    FunctionPointersTuple) \
template<typename T, typename U> \
constexpr auto Make_Function_Pointers_Nonsym() { \
  return std::make_tuple FunctionPointersTuple ; \
} \
template<typename T, typename... Us> \
constexpr auto Make_Function_Pointers_Nonsym_Pack2() { \
  return std::tuple_cat(Make_Function_Pointers_Nonsym<T, Us>()...); \
} \
/* NOLINTNEXTLINE(readability/fn_size) */ \
static constexpr auto Function_Templates_Nonsym __attribute__((used)) = \
    Make_Function_Pointers_Nonsym_Pack2< \
        ::drake::internal::scalar_phase_t<DRAKE_ONCE_PER_SCALAR_PHASE>::type, \
        double, \
        ::drake::AutoDiffXd>();
#else
#define \
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS( \
    FunctionPointersTuple) \
static_assert(true);
#endif
// clang-format on

/// @}
