#pragma once

#include <string>

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
/**
 * Return ⌈log₂(n)⌉, namely the minimal integer no smaller than log₂(n), with
 * base 2.
 * @param n A positive integer.
 * @return The minimal integer no smaller than log₂(n).
 */
constexpr int CeilLog2(int n) { return n == 1 ? 0 : 1 + CeilLog2((n + 1) / 2); }

/**
 * The size of the new binary variables in the compile time, for Special Ordered
 * Set of type 2 (SOS2) constraint. The SOS2 constraint says that
 * <pre>
 *   λ(0) + ... + λ(n) = 1
 *   ∀i. λ(i) ≥ 0
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) + λ(j + 1) = 1
 * </pre>
 * @tparam NumLambda The length of the lambda vector. NumLambda = n + 1.
 */
template <int NumLambda>
struct LogarithmicSos2NewBinaryVariables {
  static constexpr int Rows = CeilLog2(NumLambda - 1);
  typedef VectorDecisionVariable<Rows> type;
};

template <>
struct LogarithmicSos2NewBinaryVariables<Eigen::Dynamic> {
  typedef VectorXDecisionVariable type;
  static const int Rows = Eigen::Dynamic;
};

/**
 * Adds the special ordered set 2 (SOS2) constraint,
 * <pre>
 *   λ(0) + ... + λ(n) = 1
 *   ∀i. λ(i) ≥ 0
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) + λ(j + 1) = 1
 * </pre>
 * Namely at most two entries in λ can be strictly positive, and these two
 * entries have to be adjacent. All other λ should be zero. Moreover, the
 * non-zero λ satisfies
 * λ(j) + λ(j + 1) = 1.
 * We will need to add ⌈log₂(n - 1)⌉ binary variables, where n is the number of
 * rows in λ. For more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser, 2011.
 * @param prog Add the SOS2 constraint to this mathematical program.
 * @param lambda At most two entries in λ can be strictly positive, and these
 * two entries have to be adjacent. All other entries are zero.
 * @return y The newly added binary variables. The assignment of the binary
 * variable y implies which two λ can be strictly positive.
 * With a binary assignment on y, and suppose the integer M corresponds to
 * (y(0), y(1), ..., y(⌈log₂(n - 1)⌉)) in Gray code, then only λ(M) and λ(M + 1)
 * can be non-zero. For example, if the assignment of y = (1, 1), in Gray code,
 * (1, 1) represents integer 2, so only λ(2) and λ(3) can be strictly positive.
 */
template <typename Derived>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value &&
        std::is_same<typename Derived::Scalar, symbolic::Expression>::value &&
        Derived::ColsAtCompileTime == 1,
    typename LogarithmicSos2NewBinaryVariables<
        Derived::RowsAtCompileTime>::type>::type
AddLogarithmicSos2Constraint(MathematicalProgram* prog, const Derived& lambda,
                             const std::string& binary_variable_name = "y") {
  const int binary_variable_size = CeilLog2(lambda.rows() - 1);
  const auto y = prog->NewBinaryVariables<
      LogarithmicSos2NewBinaryVariables<Derived::RowsAtCompileTime>::Rows, 1>(
      binary_variable_size, 1, binary_variable_name);
  AddLogarithmicSos2Constraint(prog, lambda,
                               y.template cast<symbolic::Expression>());
  return y;
}

/** Adds the special ordered set 2 (SOS2) constraint,
 * @see AddLogarithmicSos2Constraint.
 */
void AddLogarithmicSos2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& y);

/**
 * Adds the special ordered set 2 (SOS2) constraint. y(i) takes binary values
 * (either 0 or 1).
 * <pre>
 *   y(i) = 1 => λ(i) + λ(i + 1) = 1.
 * </pre>
 * @see AddLogarithmicSos2Constraint for a complete explanation on SOS2
 * constraint.
 * @param prog The optimization program to which the SOS2 constraint is added.
 * @param lambda At most two entries in λ can be strictly positive, and these
 * two entries have to be adjacent. All other entries are zero. Moreover, these
 * two entries should sum up to 1.
 * @param y y(i) takes binary value, and determines which two entries in λ can
 * be strictly positive. Throw a runtime error if y.rows() != lambda.rows() - 1.
 */
void AddSos2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& y);

/**
 * Adds the special ordered set of type 1 (SOS1) constraint. Namely
 * <pre>
 *   λ(0) + ... + λ(n-1) = 1
 *   λ(i) ≥ 0 ∀i
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) = 1
 * </pre>
 * one and only one of λ(i) is strictly positive (equals to 1 in this case).
 * We will need to add ⌈log₂(n)⌉ binary variables, where n is the number of
 * rows in λ. For more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser, 2011.
 * @param prog The program to which the SOS1 constraint is added.
 * @param lambda lambda is in SOS1.
 * @param y The binary variables indicating which λ is positive. For a given
 * assignment on the binary variable `y`, if (y(0), ..., y(⌈log₂(n)⌉) represents
 * integer M in `codes`, then only λ(M) is positive. Namely, if
 * (y(0), ..., y(⌈log₂(n)⌉) equals to codes.row(M), then λ(M) = 1
 * @param codes A n x ⌈log₂(n)⌉ matrix. code.row(i) represents integer i.
 * No two rows of `codes` can be the same. @throws std::runtime_error if
 * @p codes has a non-binary entry (0, 1).
 */
void AddLogarithmicSos1Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::MatrixXi>& codes);

/**
 * For a continuous variable whose range is cut into small intervals, we will
 * use binary variables to represent which interval the continuous variable is
 * in. We support two representations, either using logarithmic number of binary
 * variables, or linear number of binary variables. For more details, @see
 * AddLogarithmicSos2Constraint and AddSos2Constraint
 */
enum class IntervalBinning {
  kLogarithmic,
  kLinear
};

/**
 * Constrain `w` to approximate the bilinear product x * y. We know
 * that x is in one of the intervals [φx(i), φx(i+1)], y is in one of the
 * intervals [φy(j), φy(j+1)]. The variable `w` is constrained to be in the
 * convex hull of x * y for x in [φx(i), φx(i+1)], y in [φy(j), φy(j+1)], namely
 * (x, y, w) is in the tetrahedron, with vertices [φx(i), φy(j), φx(i)*φy(j)],
 * [φx(i+1), φy(j), φx(i+1)*φy(j)], [φx(i), φy(j+1), φx(i)*φy(j+1)] and
 * [φx(i+1), φy(j+1), φx(i+1)*φy(j+1)]
 * We use two different encoding schemes on the binary variables, to determine
 * which interval is active. We can choose either linear or logarithmic binning.
 * When using linear binning, for a variable with N intervals, we
 * use N binary variables, and B(i) = 1 indicates the variable is in the i'th
 * interval. When using logarithmic binning, we use ⌈log₂(N)⌉ binary variables.
 * If these binary variables represent integer M in the reflected Gray code,
 * then the continuous variable is in the M'th interval.
 * @param prog The program to which the bilinear product constraint is added
 * @param x The decision variable.
 * @param y The decision variable.
 * @param w The expression to approximate x * y
 * @param phi_x The end points of the intervals for `x`.
 * @param phi_y The end points of the intervals for `y`.
 * @param Bx The binary variables for the interval in which x stays encoded as
 * described above.
 * @param By The binary variables for the interval in which y stays encoded as
 * described above.
 * @param binning Determine whether to use linear binning or
 * logarithmic binning.
 * @return lambda The auxiliary continuous variables.
 * x = φxᵀ * (λ.rowwise().sum())
 * y = φyᵀ * (λ.cowwise().sum())
 * w = sum_{i, j} φx(i) * φy(j) * λ(i, j)
 * Both λ.rowwise().sum() and λ.colwise().sum() satisfy SOS2 constraint.
 * If x ∈ [φx(M), φx(M+1)] and y ∈ [φy(N), φy(N+1)], then only λ(M, N),
 * λ(M + 1, N), λ(M, N + 1) and λ(M+1, N+1) can be strictly positive, all other
 * λ(i, j) are zero.
 */
template <typename DerivedPhiX, typename DerivedPhiY, typename DerivedBx,
    typename DerivedBy>
typename std::enable_if<
    is_eigen_vector_of<DerivedPhiX, double>::value &&
        is_eigen_vector_of<DerivedPhiY, double>::value &&
        is_eigen_vector_of<DerivedBx, symbolic::Expression>::value &&
        is_eigen_vector_of<DerivedBy, symbolic::Expression>::value,
    MatrixDecisionVariable<DerivedPhiX::RowsAtCompileTime,
                           DerivedPhiY::RowsAtCompileTime>>::type
AddBilinearProductMcCormickEnvelopeSos2(
    MathematicalProgram* prog, const symbolic::Variable& x,
    const symbolic::Variable& y, const symbolic::Expression& w,
    const DerivedPhiX& phi_x, const DerivedPhiY& phi_y, const DerivedBx& Bx,
    const DerivedBy& By, IntervalBinning binning) {
  switch (binning) {
    case IntervalBinning::kLogarithmic:
      DRAKE_ASSERT(Bx.rows() == CeilLog2(phi_x.rows() - 1));
      DRAKE_ASSERT(By.rows() == CeilLog2(phi_y.rows() - 1));
      break;
    case IntervalBinning::kLinear:
      DRAKE_ASSERT(Bx.rows() == phi_x.rows() - 1);
      DRAKE_ASSERT(By.rows() == phi_y.rows() - 1);
      break;
  }
  const int num_phi_x = phi_x.rows();
  const int num_phi_y = phi_y.rows();
  auto lambda = prog->NewContinuousVariables<DerivedPhiX::RowsAtCompileTime,
                                             DerivedPhiY::RowsAtCompileTime>(
      num_phi_x, num_phi_y, "lambda");

  prog->AddBoundingBoxConstraint(0, 1, lambda);

  symbolic::Expression x_convex_combination{0};
  symbolic::Expression y_convex_combination{0};
  symbolic::Expression w_convex_combination{0};
  for (int i = 0; i < num_phi_x; ++i) {
    for (int j = 0; j < num_phi_y; ++j) {
      x_convex_combination += lambda(i, j) * phi_x(i);
      y_convex_combination += lambda(i, j) * phi_y(j);
      w_convex_combination += lambda(i, j) * phi_x(i) * phi_y(j);
    }
  }
  prog->AddLinearConstraint(x == x_convex_combination);
  prog->AddLinearConstraint(y == y_convex_combination);
  prog->AddLinearConstraint(w == w_convex_combination);

  switch (binning) {
    case IntervalBinning::kLogarithmic:
      AddLogarithmicSos2Constraint(
          prog, lambda.template cast<symbolic::Expression>().rowwise().sum(),
          Bx);
      AddLogarithmicSos2Constraint(prog,
                                   lambda.template cast<symbolic::Expression>()
                                       .colwise()
                                       .sum()
                                       .transpose(),
                                   By);
      break;
    case IntervalBinning::kLinear:
      AddSos2Constraint(
          prog, lambda.template cast<symbolic::Expression>().rowwise().sum(),
          Bx);
      AddSos2Constraint(prog, lambda.template cast<symbolic::Expression>()
                                  .colwise()
                                  .sum()
                                  .transpose(),
                        By);
      break;
  }
  return lambda;
}

}  // namespace solvers
}  // namespace drake
