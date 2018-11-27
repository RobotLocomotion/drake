#pragma once

#include <type_traits>

#include <Eigen/Core>

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
}  // namespace drake
