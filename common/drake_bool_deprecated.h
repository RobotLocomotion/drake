#pragma once

// Do not include drake_bool_deprecated.h directly.
#ifndef DRAKE_BOOL_DEPRECATED_OK_TO_INCLUDE
# error Do not include drake_bool_deprecated.h directly.
#endif

// TODO(jwnimmer-tri) Delete drake_bool_deprecated.h on or about 2018-12-01.

#include <type_traits>

#include "drake/common/autodiff.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"

// Without this amendment, all of our functions yell about using themselves
// when they are deprecated.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {

template <typename T>
using scalar_predicate_t DRAKE_DEPRECATED("Use boolean<T>.") = boolean<T>;

/// Class representing a Boolean value independent of the underlying
/// scalar type T:
///
/// - For `double` or autodiff, this class embeds a `bool` value.
/// - For `symbolic::Expression`, this class embeds a `symbolic::Formula`
///   value.
///
/// When this class wraps a `bool` value (e.g. T = `double` or autodiff), this
/// class is contextually convertible to bool. For example, the following works.
///
/// @code
/// const Bool<double> b{3.0 < 4.0};
/// if (b) {
///   ...
/// }
/// @endcode
///
/// Otherwise (e.g. T = `symbolic::Expression`), this class is *not*
/// contextually convertible to bool. In this case, to convert a value to
/// `bool`, one needs to explicitly call `ExtractBoolOrThrow` defined below in
/// this file. Here is an example use-case:
///
/// @code
/// const Bool<symbolic::Expression> b{...};
/// if (ExtractBoolOrThrow(b)) {
///   ...
/// }
/// @endcode
///
/// In contrast, the following code does not compile:
///
/// @code
/// const Bool<symbolic::Expression> b{...};
/// if (b) {
///   ...
/// }
/// @endcode
///
template <typename T>
class DRAKE_DEPRECATED("Deprecated without any replacement") Bool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bool);

  using value_type = decltype(T() < T());

  /// Constructs with @p value.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Bool(const value_type& value) : value_{value} {}

  /// Constructs with a Boolean @p b.
  ///
  /// @note This constructor is only enabled if `value_type` is not `bool` in
  /// which case the above constructor, `Bool(const value_type& value)`, is
  /// used instead.
  template <typename = std::enable_if<!std::is_same<value_type, bool>::value>>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Bool(bool b)
      : value_{b ? !(T(0) < T(0)) /* True */ : T(0) < T(0) /* False */} {}

  /// Provides implicit bool conversion only if Bool<T>::value_type is bool.
  ///
  /// @note The use of std::enable_if is not allowed here. I found a workaround
  /// of using std::conditional which is explained in
  /// https://stackoverflow.com/a/19434345.
  operator typename std::conditional_t<std::is_same<value_type, bool>::value,
                                       bool, void>() const {
    return value();
  }

  /// Returns a copy of its value.
  value_type value() const { return value_; }

  /// Returns the true value.
  static Bool<T> True() { return Bool{!(T(0) < T(0))}; }

  /// Returns the false value.
  static Bool<T> False() { return Bool{T(0) < T(0)}; }

 private:
  value_type value_{};
};

/// Extracts a bool value from @p b or throws an exception if the extraction
/// fails (i.e. due to free variables in a symbolic::Formula).
template <typename T>
DRAKE_DEPRECATED("Bool<T> is deprecated.")
bool ExtractBoolOrThrow(const Bool<T>& b) {
  return bool{b.value()};
}

/// Provides logical AND operator (&&) between Bool<T> and Bool<T> when
/// Bool<T>::value_type is *not* bool.
///
/// @note This conditioning is necessary because of the implicit bool
/// conversion. For instance, if we provide operator&& for all Bool<T>, then
/// `Bool<double> && bool` becomes ambiguous because there are two possible
/// operator&&s -- one for C++ bool and another for `drake::Bool<double>`.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator&&(const Bool<T>& b1, const Bool<T>& b2) {
  // Previously, we use the "friend" trick explained in the Meyer's effective
  // C++ 3rd. (item 46) to provide a single `operator&&` definition. The trick
  // allows the definition to support not only `Bool<T> && Bool<T>` case but
  // also `Bool<T>::value && Bool<T>` and `Bool<T> && Bool<T>::value_type`
  // cases. However, because of this extra constraint `Bool<T>::value_type !=
  // bool`, we cannot use this friend trick anymore. As a result, we need to
  // provide three definitions explicitly.
  return Bool<T>{b1.value() && b2.value()};
}

/// Provides logical AND operator (&&) between Bool<T>::value_type and Bool<T>
/// when Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator&&(const typename Bool<T>::value_type& v1, const Bool<T>& b2) {
  return Bool<T>{v1 && b2.value()};
}

/// Provides logical AND operator (&&) between Bool<T> and Bool<T>::value_type
/// when Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator&&(const Bool<T>& b1, const typename Bool<T>::value_type& v2) {
  return Bool<T>{b1.value() && v2};
}

/// Provides logical OR operator (||) between Bool<T> and Bool<T> when
/// Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator||(const Bool<T>& b1, const Bool<T>& b2) {
  return Bool<T>{b1.value() || b2.value()};
}

/// Provides logical OR operator (||) between Bool<T>::value_type and Bool<T>
/// when Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator||(const typename Bool<T>::value_type& v1, const Bool<T>& b2) {
  return Bool<T>{v1 || b2.value()};
}

/// Provides logical OR operator (||) between Bool<T> and Bool<T>::value_type
/// when Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator||(const Bool<T>& b1, const typename Bool<T>::value_type& v2) {
  return Bool<T>{b1.value() || v2};
}

/// Provides logical NOT operator (!) when Bool<T>::value_type is *not* bool.
template <typename T>
std::enable_if_t<!std::is_same<typename Bool<T>::value_type, bool>::value,
                 Bool<T>>
operator!(const Bool<T>& b) {
  return Bool<T>{!b.value()};
}

// Note that we need to have `#include "drake/common/autodiff.h"` atop this
// file because, in case of T = AutoDiffXd, this template function calls
// another template function defined in common/autodiff.h. See
// https://clang.llvm.org/compatibility.html#dep_lookup for more information.
/// Allows users to use `if_then_else` with a conditional of `Bool<T>` type in
/// addition to `Bool<T>::value_type`.
template <typename T>
DRAKE_DEPRECATED("Bool<T> is deprecated")
T if_then_else(const Bool<T>& b, const T& v_then, const T& v_else) {
  return if_then_else(b.value(), v_then, v_else);
}

/// Allows users to use `cond` with conditionals of `Bool<T>` type in addition
/// to `Bool<T>::value_type`.
template <typename T, typename... Rest>
DRAKE_DEPRECATED("Bool<T> is deprecated")
T cond(const Bool<T>& b, const T& e_then, Rest... rest) {
  return cond(b.value(), e_then, rest...);
}

namespace assert {
/// Provides the specialization of ConditionTraits for `Bool<T>` so that a value
/// of Bool<T> can be passed to DRAKE_ASSERT/DRAKE_DEMAND macros.
template <typename T>
struct ConditionTraits<Bool<T>> {
  DRAKE_DEPRECATED("Bool<T> is deprecated")
  static constexpr bool is_valid =
      ConditionTraits<typename Bool<T>::value_type>::is_valid;
  static bool Evaluate(const Bool<T>& b) {
    return ConditionTraits<typename Bool<T>::value_type>::Evaluate(b.value());
  }
};

}  // namespace assert
}  // namespace drake

#pragma GCC diagnostic pop  // "-Wdeprecated-declarations"
