#pragma once

#include <limits>
#include <memory>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
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
 * arguments.  Constraints of the form x == 1 (which could be created as a
 * BoundingBoxConstraint or LinearEqualityConstraint) will be
 * constructed as a LinearEqualityConstraint.
 */
Binding<Constraint> ParseConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub);

/*
 * Assist MathematicalProgram::AddLinearConstraint(...).
 */
inline Binding<Constraint> ParseConstraint(
    const symbolic::Expression& e, const double lb, const double ub) {
  return ParseConstraint(Vector1<symbolic::Expression>(e), Vector1<double>(lb),
                         Vector1<double>(ub));
}

/**
 * Parses the constraint lb <= e <= ub to linear constraint types, including
 * BoundingBoxConstraint, LinearEqualityConstraint, and LinearConstraint. If @p
 * e is not a linear expression, then return a null pointer.
 * If the constraint lb <= e <= ub can be parsed as a BoundingBoxConstraint,
 * then we return a BoundingBoxConstraint pointer. For example, the constraint
 * 1 <= 2 * x + 3 <= 4 is equivalent to the bounding box constraint -1 <= x <=
 * 0.5. Hence we will return the BoundingBoxConstraint in this case.
 */
std::unique_ptr<Binding<Constraint>> MaybeParseLinearConstraint(
    const symbolic::Expression& e, double lb, double ub);

/*
 * Assist MathematicalProgram::AddLinearConstraint(...).
 */
Binding<Constraint> ParseConstraint(const symbolic::Formula& f);

/*
 * Assist MathematicalProgram::AddLinearConstraint(...).
 */
Binding<Constraint> ParseConstraint(
    const std::set<symbolic::Formula>& formulas);

/*
 * Assist MathematicalProgram::AddLinearConstraint(...).
 */
template <typename Derived>
typename std::enable_if_t<
    is_eigen_scalar_same<Derived, symbolic::Formula>::value,
    Binding<Constraint>>
ParseConstraint(const Eigen::ArrayBase<Derived>& formulas) {
  const auto n = formulas.rows() * formulas.cols();

  // Decomposes 2D-array of formulas into 1D-vector of expression, `v`, and
  // two 1D-vector of double `lb` and `ub`.
  constexpr int flat_vector_size{
      MultiplyEigenSizes<Derived::RowsAtCompileTime,
                         Derived::ColsAtCompileTime>::value};
  Eigen::Matrix<symbolic::Expression, flat_vector_size, 1> v{n};
  Eigen::Matrix<double, flat_vector_size, 1> lb{n};
  Eigen::Matrix<double, flat_vector_size, 1> ub{n};
  int k{0};  // index variable for 1D components.
  for (int j{0}; j < formulas.cols(); ++j) {
    for (int i{0}; i < formulas.rows(); ++i) {
      const symbolic::Formula& f{formulas(i, j)};
      if (is_equal_to(f)) {
        // f(i) := (lhs == rhs)
        //         (lhs - rhs == 0)
        v(k) = get_lhs_expression(f) - get_rhs_expression(f);
        lb(k) = 0.0;
        ub(k) = 0.0;
      } else if (is_less_than_or_equal_to(f)) {
        // f(i) := (lhs <= rhs)
        //         (-∞ <= lhs - rhs <= 0)
        v(k) = get_lhs_expression(f) - get_rhs_expression(f);
        lb(k) = -std::numeric_limits<double>::infinity();
        ub(k) = 0.0;
      } else if (is_greater_than_or_equal_to(f)) {
        // f(i) := (lhs >= rhs)
        //         (∞ >= lhs - rhs >= 0)
        v(k) = get_lhs_expression(f) - get_rhs_expression(f);
        lb(k) = 0.0;
        ub(k) = std::numeric_limits<double>::infinity();
      } else {
        std::ostringstream oss;
        oss << "ParseConstraint is called with an "
               "array of formulas which includes a formula "
            << f
            << " which is not a relational formula using one of {==, <=, >=} "
               "operators.";
        throw std::runtime_error(oss.str());
      }
      k++;
    }
  }
  return ParseConstraint(v, lb, ub);
}

/*
 * Assist functionality for ParseLinearEqualityConstraint(...).
 */
Binding<LinearEqualityConstraint> DoParseLinearEqualityConstraint(
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
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const std::set<symbolic::Formula>& formulas);

/*
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
Binding<LinearEqualityConstraint> ParseLinearEqualityConstraint(
    const symbolic::Formula& f);

/*
 * Assist MathematicalProgram::AddLinearEqualityConstraint(...).
 */
template <typename DerivedV, typename DerivedB>
typename std::enable_if_t<
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
typename std::enable_if_t<
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
Binding<QuadraticConstraint> ParseQuadraticConstraint(
    const symbolic::Expression& e, double lower_bound, double upper_bound);

/*
 * Assist MathematicalProgram::AddPolynomialConstraint(...).
 * @note Non-symbolic, but this seems to have a separate purpose than general
 * construction.
 */
std::shared_ptr<Constraint> MakePolynomialConstraint(
    const VectorXPoly& polynomials,
    const std::vector<Polynomiald::VarType>& poly_vars,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

/*
 * Assist MathematicalProgram::AddLorentzConeConstraint(...).
 */
Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
    LorentzConeConstraint::EvalType eval_type =
        LorentzConeConstraint::EvalType::kConvexSmooth);

/*
 * Assist MathematicalProgram::AddLorentzConeConstraint(...).
 */
Binding<LorentzConeConstraint> ParseLorentzConeConstraint(
    const symbolic::Expression& linear_expr,
    const symbolic::Expression& quadratic_expr, double tol = 0,
    LorentzConeConstraint::EvalType eval_type =
        LorentzConeConstraint::EvalType::kConvexSmooth);

/*
 * Assist MathematicalProgram::AddRotatedLorentzConeConstraint(...)
 */
Binding<RotatedLorentzConeConstraint> ParseRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

/*
 * Assist MathematicalProgram::AddRotatedLorentzConeConstraint(...)
 */
Binding<RotatedLorentzConeConstraint> ParseRotatedLorentzConeConstraint(
    const symbolic::Expression& linear_expr1,
    const symbolic::Expression& linear_expr2,
    const symbolic::Expression& quadratic_expr, double tol = 0);

// TODO(eric.cousineau): Implement this if variable creation is separated.
// Format would be (tuple(linear_binding, psd_binding), new_vars)
// ParsePositiveSemidefiniteConstraint(
//     const Eigen::Ref<MatrixX<symbolic::Expression>>& e) {
//   // ...
//   return std::make_tuple(linear_binding, psd_binding);
// }

template <typename Derived>
typename std::enable_if_t<is_eigen_vector_of<Derived, symbolic::Formula>::value,
                          Binding<Constraint>>
ParseConstraint(const Eigen::MatrixBase<Derived>&) {
  // TODO(eric.cousineau): Implement this.
  throw std::runtime_error("Not implemented");
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
