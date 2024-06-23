#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {

namespace internal {

// TODO(eric.cousineau): Use Eigen::Ref more pervasively when no temporaries
// are allocated (or if it doesn't matter if they are).

/**
 * The resulting constraint may be a BoundingBoxConstraint, LinearConstraint,
 * LinearEqualityConstraint, or ExpressionConstraint, depending on the
 * arguments. Constraints of the form x == 1 (which could be created as a
 * BoundingBoxConstraint or LinearEqualityConstraint) will be
 * constructed as a LinearEqualityConstraint.
 */
[[nodiscard]] Binding<Constraint> ParseConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub);

/*
 * Assist MathematicalProgram::AddLinearConstraint(...).
 */
[[nodiscard]] inline Binding<Constraint> ParseConstraint(
    const symbolic::Expression& e, const double lb, const double ub) {
  return ParseConstraint(Vector1<symbolic::Expression>(e), Vector1<double>(lb),
                         Vector1<double>(ub));
}

/**
 * Parses the constraint lb <= e <= ub to linear constraint types, including
 * BoundingBoxConstraint, LinearEqualityConstraint, and LinearConstraint. If @p
 * e is not a linear expression, then returns a null pointer.
 * If the constraint lb <= e <= ub can be parsed as a BoundingBoxConstraint,
 * then we return a BoundingBoxConstraint pointer. For example, the constraint
 * 1 <= 2 * x + 3 <= 4 is equivalent to the bounding box constraint -1 <= x <=
 * 0.5. Hence we will return the BoundingBoxConstraint in this case.
 */
[[nodiscard]] std::unique_ptr<Binding<Constraint>> MaybeParseLinearConstraint(
    const symbolic::Expression& e, double lb, double ub);

/*
 * Creates a constraint that should satisfy the formula `f`.
 * @throws exception if `f` is always false (for example 1 >= 2).
 * @note if `f` is always true, then returns an empty BoundingBoxConstraint
 * binding.
 */
[[nodiscard]] Binding<Constraint> ParseConstraint(const symbolic::Formula& f);

/*
 * Creates a constraint that enforces all `formulas` to be satisfied.
 * @throws exception if any of `formulas` is always false (for example 1 >= 2).
 * @note If any entry in `formulas` is always true, then that entry is ignored.
 * If all entries in `formulas` are true, then returns an empty
 * BoundingBoxConstraint binding.
 */
[[nodiscard]] Binding<Constraint> ParseConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Formula>>& formulas);

/*
 * Assist functionality for ParseLinearEqualityConstraint(...).
 */
[[nodiscard]] Binding<LinearEqualityConstraint> DoParseLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b);

/*
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
inline Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const symbolic::Expression& e, double b) {
  return DoParseLinearEqualityConstraint(Vector1<symbolic::Expression>(e),
                                         Vector1d(b));
}

/*
 * Creates a constraint to satisfy all entries in `formulas`.
 * @throws exception if any of `formulas` is always false (for example 1 == 2)
 * @note If any entry in `formulas` is always true, then that entry is ignored;
 * if all entries in `formulas` are always true, then returns an empty linear
 * equality constraint binding.
 */
[[nodiscard]] Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const std::set<symbolic::Formula>& formulas);

/*
 *
 * Creates a linear equality constraint satisfying the formula `f`.
 * @throws exception if `f` is always false (for example 1 == 2)
 * @note if `f` is always true, then returns an empty linear equality constraint
 * binding.
 */
[[nodiscard]] Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const symbolic::Formula& f);

/*
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
template <typename DerivedV, typename DerivedB>
[[nodiscard]] typename std::enable_if_t<
    is_eigen_vector_expression_double_pair<DerivedV, DerivedB>::value,
    Binding<LinearEqualityConstraint>>
ParseLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                              const Eigen::MatrixBase<DerivedB>& b) {
  return DoParseLinearEqualityConstraint(V, b);
}

/*
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
template <typename DerivedV, typename DerivedB>
[[nodiscard]] typename std::enable_if_t<
    is_eigen_nonvector_expression_double_pair<DerivedV, DerivedB>::value,
    Binding<LinearEqualityConstraint>>
ParseLinearEqualityConstraint(const Eigen::MatrixBase<DerivedV>& V,
                              const Eigen::MatrixBase<DerivedB>& B,
                              bool lower_triangle = false) {
  if (lower_triangle) {
    DRAKE_DEMAND(V.rows() == V.cols() && B.rows() == B.cols());
  }
  DRAKE_DEMAND(V.rows() == B.rows() && V.cols() == B.cols());

  // Form the flatten version of V and B, when lower_triangle = false,
  // the flatten version is just to concatenate each column of the matrix;
  // otherwise the flatten version is to concatenate each column of the
  // lower triangular part of the matrix.
  const int V_rows = DerivedV::RowsAtCompileTime != Eigen::Dynamic
                         ? static_cast<int>(DerivedV::RowsAtCompileTime)
                         : static_cast<int>(DerivedB::RowsAtCompileTime);
  const int V_cols = DerivedV::ColsAtCompileTime != Eigen::Dynamic
                         ? static_cast<int>(DerivedV::ColsAtCompileTime)
                         : static_cast<int>(DerivedB::ColsAtCompileTime);

  if (lower_triangle) {
    constexpr int V_triangular_size =
        V_rows != Eigen::Dynamic ? (V_rows + 1) * V_rows / 2 : Eigen::Dynamic;
    int V_triangular_size_dynamic = V.rows() * (V.rows() + 1) / 2;
    Eigen::Matrix<symbolic::Expression, V_triangular_size, 1> flat_lower_V(
        V_triangular_size_dynamic);
    Eigen::Matrix<double, V_triangular_size, 1> flat_lower_B(
        V_triangular_size_dynamic);
    int V_idx = 0;
    for (int j = 0; j < V.cols(); ++j) {
      for (int i = j; i < V.rows(); ++i) {
        flat_lower_V(V_idx) = V(i, j);
        flat_lower_B(V_idx) = B(i, j);
        ++V_idx;
      }
    }
    return DoParseLinearEqualityConstraint(flat_lower_V, flat_lower_B);
  } else {
    const int V_size = V_rows != Eigen::Dynamic && V_cols != Eigen::Dynamic
                           ? V_rows * V_cols
                           : Eigen::Dynamic;
    Eigen::Matrix<symbolic::Expression, V_size, 1> flat_V(V.size());
    Eigen::Matrix<double, V_size, 1> flat_B(V.size());
    int V_idx = 0;
    for (int j = 0; j < V.cols(); ++j) {
      for (int i = 0; i < V.rows(); ++i) {
        flat_V(V_idx) = V(i, j);
        flat_B(V_idx) = B(i, j);
        ++V_idx;
      }
    }
    return DoParseLinearEqualityConstraint(flat_V, flat_B);
  }
}

/**
 * Assists MathematicalProgram::AddConstraint(...) to create a quadratic
 * constraint binding.
 */
[[nodiscard]] Binding<QuadraticConstraint> ParseQuadraticConstraint(
    const symbolic::Expression& e, double lower_bound, double upper_bound,
    std::optional<QuadraticConstraint::HessianType> hessian_type =
        std::nullopt);

/*
 * Assist MathematicalProgram::AddPolynomialConstraint(...).
 * @note Non-symbolic, but this seems to have a separate purpose than general
 * construction.
 */
[[nodiscard]] std::shared_ptr<Constraint> MakePolynomialConstraint(
    const VectorXPoly& polynomials,
    const std::vector<Polynomiald::VarType>& poly_vars,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/*
 * Assist MathematicalProgram::AddLorentzConeConstraint(...).
 */
[[nodiscard]] Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const symbolic::Formula& f,
    LorentzConeConstraint::EvalType eval_type =
        LorentzConeConstraint::EvalType::kConvexSmooth,
    double psd_tol = 1e-8, double coefficient_tol = 1e-8);

/*
 * Assist MathematicalProgram::AddLorentzConeConstraint(...).
 */
[[nodiscard]] Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    LorentzConeConstraint::EvalType eval_type =
        LorentzConeConstraint::EvalType::kConvexSmooth);

/*
 * Assist MathematicalProgram::AddLorentzConeConstraint(...).
 */
[[nodiscard]] Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const symbolic::Expression& linear_expr,
    const symbolic::Expression& quadratic_expr, double tol = 0,
    LorentzConeConstraint::EvalType eval_type =
        LorentzConeConstraint::EvalType::kConvexSmooth);

/*
 * Assist MathematicalProgram::AddRotatedLorentzConeConstraint(...)
 */
[[nodiscard]] Binding<RotatedLorentzConeConstraint>
ParseRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

/*
 * Assist MathematicalProgram::AddRotatedLorentzConeConstraint(...)
 */
[[nodiscard]] Binding<RotatedLorentzConeConstraint>
ParseRotatedLorentzConeConstraint(const symbolic::Expression& linear_expr1,
                                  const symbolic::Expression& linear_expr2,
                                  const symbolic::Expression& quadratic_expr,
                                  double tol = 0);

/** For a convex quadratic constraint 0.5xᵀQx + bᵀx + c <= 0, we parse it as a
 * rotated Lorentz cone constraint [-bᵀx-c, 1, Fx] is in the rotated Lorentz
 * cone where FᵀF = 0.5 * Q
 * @param zero_tol The tolerance to determine if Q is a positive semidefinite
 * matrix. Check math::DecomposePSDmatrixIntoXtransposeTimesX for a detailed
 * explanation. zero_tol should be non-negative. @default is 0.
 * @throw exception if this quadratic constraint is not convex (Q is not
 * positive semidefinite)
 *
 * You could refer to
 * https://docs.mosek.com/latest/pythonapi/advanced-toconic.html for derivation.
 */
[[nodiscard]] std::shared_ptr<RotatedLorentzConeConstraint>
ParseQuadraticAsRotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c, double zero_tol = 0.);

// TODO(eric.cousineau): Implement this if variable creation is separated.
// Format would be (tuple(linear_binding, psd_binding), new_vars)
// ParsePositiveSemidefiniteConstraint(
//     const Eigen::Ref<MatrixX<symbolic::Expression>>& e) {
//   // ...
//   return std::make_tuple(linear_binding, psd_binding);
// }

}  // namespace internal
}  // namespace solvers
}  // namespace drake
