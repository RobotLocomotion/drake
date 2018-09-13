#pragma once

#include <type_traits>

#include <Eigen/Core>

#include "drake/common/autodiff.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"

namespace drake {

/// A traits struct that describes the return type of predicates over a scalar
/// type (named `T`).  For example, a predicate that evaluates `double`s will
/// return a `bool`, but a predicate that evaluates symbolic::Expression will
/// return a symbolic::Formula.  By default, the return type is inferred from
/// the type's comparison operator, but scalar types are permitted to
/// specialize this template for their needs.
template <typename T>
struct scalar_predicate {
  /// The return type of predicates over T.
  using type = decltype(T() < T());

  /// Whether `type` is `bool`.
  static constexpr bool is_bool = std::is_same<type, bool>::value;
};

/// An alias for a boolean-like value, conditioned on the scalar type `T`.
/// In many cases this will be a synonym for `bool`, e.g., when `T = double`.
/// When `T = symbolic::Expression`, this is a synonym for `symbolic::Formula`.
/// This is a convenience abbreviation for scalar_predicate<T>::type.
template <typename T>
using boolean = typename scalar_predicate<T>::type;

template <typename T>
using scalar_predicate_t DRAKE_DEPRECATED("Use boolean<T>.") = boolean<T>;

/// Class representing a Boolean value independent of the underlying
/// scalar type T:
///  - For `double` or autodiff, this class embeds a `bool` value.
///  - For `symbolic::Expression`, this class embeds a `symbolic::Formula`
///    value.
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

/// Checks truth for all elements in matrix @p m.  This is identical to
/// `Eigen::DenseBase::all()`, except this function allows for lazy evaluation,
/// so works even when scalar_predicate<>::is_bool does not hold.  An empty
/// matrix returns true.
template <typename Derived>
typename Derived::Scalar all(const Eigen::DenseBase<Derived>& m) {
  using Boolish = typename Derived::Scalar;
  if (m.rows() == 0 || m.cols() == 0) {
    // `all` holds vacuously when there is nothing to check.
    return Boolish{true};
  }
  return m.redux([](const Boolish& v1, const Boolish& v2) { return v1 && v2; });
}

/// Checks if unary predicate @p pred holds for all elements in the matrix @p m.
/// An empty matrix returns true.
template <typename Derived>
boolean<typename Derived::Scalar> all_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<boolean<typename Derived::Scalar>(
        const typename Derived::Scalar&)>& pred) {
  return all(m.unaryExpr(pred));
}

/// Checks truth for at least one element in matrix @p m.  This is identical to
/// `Eigen::DenseBase::any()`, except this function allows for lazy evaluation,
/// so works even when scalar_predicate<>::is_bool does not hold.  An empty
/// matrix returns false.
template <typename Derived>
typename Derived::Scalar any(const Eigen::DenseBase<Derived>& m) {
  using Boolish = typename Derived::Scalar;
  if (m.rows() == 0 || m.cols() == 0) {
    // `any` is vacuously false when there is nothing to check.
    return Boolish{false};
  }
  return m.redux([](const Boolish& v1, const Boolish& v2) { return v1 || v2; });
}

/// Checks if unary predicate @p pred holds for at least one element in the
/// matrix @p m.  An empty matrix returns false.
template <typename Derived>
boolean<typename Derived::Scalar> any_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<boolean<typename Derived::Scalar>(
        const typename Derived::Scalar&)>& pred) {
  return any(m.unaryExpr(pred));
}

/// Checks that no elements of @p m are true.  An empty matrix returns true.
template <typename Derived>
typename Derived::Scalar none(const Eigen::MatrixBase<Derived>& m) {
  using Boolish = typename Derived::Scalar;
  const auto negate = [](const Boolish& v) -> Boolish { return !v; };
  return all(m.unaryExpr(negate));
}

/// Checks if unary predicate @p pred holds for no elements in the matrix @p m.
/// An empty matrix returns true.
template <typename Derived>
boolean<typename Derived::Scalar> none_of(
    const Eigen::MatrixBase<Derived>& m,
    const std::function<boolean<typename Derived::Scalar>(
        const typename Derived::Scalar&)>& pred) {
  return none(m.unaryExpr(pred));
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
