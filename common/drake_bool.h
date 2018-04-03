#pragma once

#include <type_traits>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {

/// Class representing a boolean value independent of the underlying
/// scalar type T:
///  - For `double` or autodiff, this class embeds a `bool` value.
///  - For `symbolic::Expression`, this class embeds a `symbolic::Formula`
///    value.
///
/// A value of this class is *not* contextually convertible to bool. To convert
/// a value to `bool`, one needs to explicitly call `ExtractBoolOrThrow` defined
/// below in this file. Here is an example use-case:
///
/// @code
/// const Bool<double> b{3.0 < 4.0};
/// if (ExtractBoolOrThrow(b)) {
///   ...
/// }
/// @endcode
///
/// In contrast, the following code does not compile:
///
/// @code
/// const Bool<double> b{3.0 < 4.0};
/// if (b) {
///   ...
/// }
/// @endcode
///
template <typename T>
class Bool {
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

  /// Returns a copy of its value.
  value_type value() const { return value_; }

  /// Returns the true value.
  static Bool<T> True() { return Bool{!(T(0) < T(0))}; }

  /// Returns the false value.
  static Bool<T> False() { return Bool{T(0) < T(0)}; }

  /// Provides logical AND operator (&&).
  ///
  /// @note We define this operator in the class as a friend function so that
  /// implicit conversion works as expected (i.e. Bool<T> &&
  /// Bool<T>::value_type). See item 46 of Effective C++ (3rd ed.) for more
  /// information.
  friend Bool<T> operator&&(const Bool<T>& b1, const Bool<T>& b2) {
    return Bool<T>{b1.value() && b2.value()};
  }

  /// Provides logical OR operator (||).
  friend Bool<T> operator||(const Bool<T>& b1, const Bool<T>& b2) {
    return Bool<T>{b1.value() || b2.value()};
  }

 private:
  value_type value_{};
};

/// Extracts a bool value from @p b or throws an exception if the extraction
/// fails (i.e. due to free variables in a symbolic::Formula).
template <typename T>
bool ExtractBoolOrThrow(const Bool<T>& b) {
  return bool{b.value()};
}

/// Provides logical NOT operator (!).
template <typename T>
Bool<T> operator!(const Bool<T>& b) {
  return Bool<T>{!b.value()};
}

/// Allows users to use `if_then_else` with a conditional of `Bool<T>` type in
/// addition to `Bool<T>::value_type`.
///
/// Note that we need to have `#include "drake/common/autodiff.h"` atop this
/// file because, in case of T = AutoDiffXd, this template function calls
/// another template function defined in common/autodiff.h. See
/// https://clang.llvm.org/compatibility.html#dep_lookup for more information.
template <typename T>
T if_then_else(const Bool<T>& b, const T& v_then, const T& v_else) {
  return if_then_else(b.value(), v_then, v_else);
}

/// Allows users to use `cond` with conditionals of `Bool<T>` type in addition
/// to `Bool<T>::value_type`.
template <typename T, typename... Rest>
T cond(const Bool<T>& b, const T& e_then, Rest... rest) {
  return cond(b.value(), e_then, rest...);
}

/// Checks if unary predicate @p pred holds for all elements in the matrix @p m.
template <typename Derived>
Bool<typename Derived::Scalar> all_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<typename Bool<typename Derived::Scalar>::value_type(
        const typename Derived::Scalar&)>& pred) {
  using T = typename Derived::Scalar;
  if (m.rows() == 0 || m.cols() == 0) {
    // all_of holds vacuously when there is nothing to check.
    return Bool<T>::True();
  }
  return m.unaryExpr(pred).redux(
      [](const typename Bool<T>::value_type& v1,
         const typename Bool<T>::value_type& v2) { return v1 && v2; });
}

/// Checks if unary predicate @p pred holds for at least one element in the
/// matrix @p m.
template <typename Derived>
Bool<typename Derived::Scalar> any_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<typename Bool<typename Derived::Scalar>::value_type(
        const typename Derived::Scalar&)>& pred) {
  using T = typename Derived::Scalar;
  if (m.rows() == 0 || m.cols() == 0) {
    // any_of is vacuously false when there is nothing to check.
    return Bool<T>::False();
  }
  return m.unaryExpr(pred).redux(
      [](const typename Bool<T>::value_type& v1,
         const typename Bool<T>::value_type& v2) { return v1 || v2; });
}

/// Checks if unary predicate @p pred holds for no elements in the matrix @p m.
template <typename Derived>
Bool<typename Derived::Scalar> none_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<typename Bool<typename Derived::Scalar>::value_type(
        const typename Derived::Scalar&)>& pred) {
  using T = typename Derived::Scalar;
  if (m.rows() == 0 || m.cols() == 0) {
    // none_of holds vacuously when there is nothing to check.
    return Bool<T>::True();
  }
  const auto neg_pred = [&pred](const T& v) { return !pred(v); };
  return m.unaryExpr(neg_pred).redux(
      [](const typename Bool<T>::value_type& v1,
         const typename Bool<T>::value_type& v2) { return v1 && v2; });
}

namespace assert {
/// Provides the specialization of ConditionTraits for `Bool<T>` so that a value
/// of Bool<T> can be passed to DRAKE_ASSERT/DRAKE_DEMAND macros.
template <typename T>
struct ConditionTraits<Bool<T>> {
  static constexpr bool is_valid =
      ConditionTraits<typename Bool<T>::value_type>::is_valid;
  static bool Evaluate(const Bool<T>& b) {
    return ConditionTraits<typename Bool<T>::value_type>::Evaluate(b.value());
  }
};
}  // namespace assert
}  // namespace drake
