#pragma once

#include <string>
#include <utility>

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
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(i) = 0 if i ≠ j and i ≠ j + 1
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
typename std::enable_if_t<
    drake::is_eigen_vector_of<Derived, symbolic::Expression>::value,
    typename LogarithmicSos2NewBinaryVariables<
        Derived::RowsAtCompileTime>::type>
AddLogarithmicSos2Constraint(MathematicalProgram* prog,
                             const Eigen::MatrixBase<Derived>& lambda,
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
 * where one and only one of λ(i) is 1, all other λ(j) are 0.
 * We will need to add ⌈log₂(n)⌉ binary variables, where n is the number of
 * rows in λ. For more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser, 2011.
 * @param prog The program to which the SOS1 constraint is added.
 * @param lambda lambda is in SOS1.
 * @param y The binary variables indicating which λ is positive. For a given
 * assignment on the binary variable `y`, if (y(0), ..., y(⌈log₂(n)⌉) represents
 * integer M in `binary_encoding`, then only λ(M) is positive. Namely, if
 * (y(0), ..., y(⌈log₂(n)⌉) equals to binary_encoding.row(M), then λ(M) = 1
 * @param binary_encoding A n x ⌈log₂(n)⌉ matrix. binary_encoding.row(i)
 * represents integer i. No two rows of `binary_encoding` can be the same.
 * @throws std::exception if @p binary_encoding has a non-binary entry (0,
 * 1).
 */
void AddLogarithmicSos1Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::MatrixXi>& binary_encoding);

/**
 * Adds the special ordered set of type 1 (SOS1) constraint. Namely
 * <pre>
 *   λ(0) + ... + λ(n-1) = 1
 *   λ(i) ≥ 0 ∀i
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) = 1
 * </pre>
 * where one and only one of λ(i) is 1, all other λ(j) are 0.
 * We will need to add ⌈log₂(n)⌉ binary variables, where n is the number of
 * rows in λ. For more information, please refer to
 *   Modeling Disjunctive Constraints with a Logarithmic Number of Binary
 *   Variables and Constraints
 *   by J. Vielma and G. Nemhauser, 2011.
 * @param prog The program to which the SOS1 constraint is added.
 * @param num_lambda n in the documentation above.
 * @return (lambda, y) lambda is λ in the documentation above. Notice that
 * λ are declared as continuous variables, but they only admit binary
 * solutions. y are binary variables of size ⌈log₂(n)⌉.
 * When this sos1 constraint is satisfied, suppose that
 * λ(i)=1 and λ(j)=0 ∀ j≠i, then y is the Reflected Gray code of i. For example,
 * suppose n = 8, i = 5, then y is a vector of size ⌈log₂(n)⌉ = 3, and the value
 * of y is (1, 1, 0) which equals to 5 according to reflected Gray code.
 */
std::pair<VectorX<symbolic::Variable>, VectorX<symbolic::Variable>>
AddLogarithmicSos1Constraint(MathematicalProgram* prog, int num_lambda);

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

std::string to_string(IntervalBinning interval_binning);

std::ostream& operator<<(std::ostream& os, const IntervalBinning& binning);

/**
 * Add constraints to the optimization program, such that the bilinear product
 * x * y is approximated by w, using Special Ordered Set of Type 2 (sos2)
 * constraint.
 * To do so, we assume that the range of x is [x_min, x_max], and the range of y
 * is [y_min, y_max]. We first consider two arrays φˣ, φʸ, satisfying
 * ```
 * x_min = φˣ₀ < φˣ₁ < ... < φˣₘ = x_max
 * y_min = φʸ₀ < φʸ₁ < ... < φʸₙ = y_max
 * ```
 * , and divide the range of x into intervals
 * [φˣ₀, φˣ₁], [φˣ₁, φˣ₂], ... , [φˣₘ₋₁, φˣₘ]
 * and the range of y into intervals
 * [φʸ₀, φʸ₁], [φʸ₁, φʸ₂], ... , [φʸₙ₋₁, φʸₙ]. The xy plane is thus cut into
 * rectangles, with each rectangle as
 * [φˣᵢ, φˣᵢ₊₁] x [φʸⱼ, φʸⱼ₊₁]. The convex hull of the surface
 * z = x * y for x, y in each rectangle is a tetrahedron. We then approximate
 * the bilinear product x * y with w, such that (x, y, w) is in one of the
 * tetrahedrons.
 *
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
 *
 * The constraints we impose are
 * ```
 * x = (φˣ)ᵀ * ∑ⱼ λᵢⱼ
 * y = (φʸ)ᵀ * ∑ᵢ λᵢⱼ
 * w = ∑ᵢⱼ φˣᵢ * φʸⱼ * λᵢⱼ
 * Both ∑ⱼ λᵢⱼ = λ.rowwise().sum() and ∑ᵢ λᵢⱼ = λ.colwise().sum() satisfy SOS2
 * constraint.
 * ```
 *
 * If x ∈ [φx(M), φx(M+1)] and y ∈ [φy(N), φy(N+1)], then only λ(M, N),
 * λ(M + 1, N), λ(M, N + 1) and λ(M+1, N+1) can be strictly positive, all other
 * λ(i, j) are zero.
 *
 * @note We DO NOT add the constraint
 * Bx(i) ∈ {0, 1}, By(j) ∈ {0, 1}
 * in this function. It is the user's responsibility to ensure that these
 * constraints are enforced.
 */
template <typename DerivedPhiX, typename DerivedPhiY, typename DerivedBx,
    typename DerivedBy>
  typename std::enable_if_t<
    is_eigen_vector_of<DerivedPhiX, double>::value &&
        is_eigen_vector_of<DerivedPhiY, double>::value &&
        is_eigen_vector_of<DerivedBx, symbolic::Expression>::value &&
        is_eigen_vector_of<DerivedBy, symbolic::Expression>::value,
    MatrixDecisionVariable<DerivedPhiX::RowsAtCompileTime,
                           DerivedPhiY::RowsAtCompileTime>>
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

/**
 * Add constraints to the optimization program, such that the bilinear product
 * x * y is approximated by w, using Mixed Integer constraint with "Multiple
 * Choice" model.
 * To do so, we assume that the range of x is [x_min, x_max], and the range of y
 * is [y_min, y_max]. We first consider two arrays φˣ, φʸ, satisfying
 * ```
 * x_min = φˣ₀ < φˣ₁ < ... < φˣₘ = x_max
 * y_min = φʸ₀ < φʸ₁ < ... < φʸₙ = y_max
 * ```
 * , and divide the range of x into intervals
 * [φˣ₀, φˣ₁], [φˣ₁, φˣ₂], ... , [φˣₘ₋₁, φˣₘ]
 * and the range of y into intervals
 * [φʸ₀, φʸ₁], [φʸ₁, φʸ₂], ... , [φʸₙ₋₁, φʸₙ]. The xy plane is thus cut into
 * rectangles, with each rectangle as
 * [φˣᵢ, φˣᵢ₊₁] x [φʸⱼ, φʸⱼ₊₁]. The convex hull of the surface
 * z = x * y for x, y in each rectangle is a tetrahedron. We then approximate
 * the bilinear product x * y with w, such that (x, y, w) is in one of the
 * tetrahedrons.
 * @param prog The optimization problem to which the constraints will be added.
 * @param x A variable in the bilinear product.
 * @param y A variable in the bilinear product.
 * @param w The expression that approximates the bilinear product x * y.
 * @param phi_x φˣ in the documentation above. Will be used to cut the range of
 * x into small intervals.
 * @param phi_y φʸ in the documentation above. Will be used to cut the range of
 * y into small intervals.
 * @param Bx The binary-valued expression indicating which interval x is in.
 * Bx(i) = 1 => φˣᵢ ≤ x ≤ φˣᵢ₊₁.
 * @param By The binary-valued expression indicating which interval y is in.
 * By(i) = 1 => φʸⱼ ≤ y ≤ φʸⱼ₊₁.
 *
 * One formulation of the constraint is
 * ```
 * x = ∑ᵢⱼ x̂ᵢⱼ
 * y = ∑ᵢⱼ ŷᵢⱼ
 * Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ
 * ∑ᵢⱼ Bˣʸᵢⱼ = 1
 * φˣᵢ Bˣʸᵢⱼ ≤ x̂ᵢⱼ ≤ φˣᵢ₊₁ Bˣʸᵢⱼ
 * φʸⱼ Bˣʸᵢⱼ ≤ ŷᵢⱼ ≤ φʸⱼ₊₁ Bˣʸᵢⱼ
 * w ≥ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ   + φˣᵢ   ŷᵢⱼ - φˣᵢ  φʸⱼ   Bˣʸᵢⱼ)
 * w ≥ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ₊₁ + φˣᵢ₊₁ ŷᵢⱼ - φˣᵢ₊₁ φʸⱼ₊₁ Bˣʸᵢⱼ)
 * w ≤ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ   + φˣᵢ₊₁ ŷᵢⱼ - φˣᵢ₊₁ φʸⱼ   Bˣʸᵢⱼ)
 * w ≤ ∑ᵢⱼ (x̂ᵢⱼ φʸⱼ₊₁ + φˣᵢ   ŷᵢⱼ - φˣᵢ   φʸⱼ₊₁ Bˣʸᵢⱼ)
 * ```
 *
 * The "logical and" constraint Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ can be imposed as
 * ```
 * Bˣʸᵢⱼ ≥ Bˣᵢ + Bʸⱼ - 1
 * Bˣʸᵢⱼ ≤ Bˣᵢ
 * Bˣʸᵢⱼ ≤ Bʸⱼ
 * 0 ≤ Bˣʸᵢⱼ ≤ 1
 * ```
 * This formulation will introduce slack variables x̂, ŷ and Bˣʸ, in total
 * 3 * m * n variables.
 *
 * In order to reduce the number of slack variables, we can further simplify
 * these constraints, by defining two vectors `x̅ ∈ ℝⁿ`, `y̅ ∈ ℝᵐ` as
 * ```
 * x̅ⱼ = ∑ᵢ x̂ᵢⱼ
 * y̅ᵢ = ∑ⱼ ŷᵢⱼ
 * ```
 * and the constraints above can be re-formulated using `x̅` and `y̅` as
 * ```
 * x = ∑ⱼ x̅ⱼ
 * y = ∑ᵢ y̅ᵢ
 * Bˣʸᵢⱼ = Bˣᵢ ∧ Bʸⱼ
 * ∑ᵢⱼ Bˣʸᵢⱼ = 1
 * ∑ᵢ φˣᵢ Bˣʸᵢⱼ ≤ x̅ⱼ ≤ ∑ᵢ φˣᵢ₊₁ Bˣʸᵢⱼ
 * ∑ⱼ φʸⱼ Bˣʸᵢⱼ ≤ y̅ᵢ ≤ ∑ⱼ φʸⱼ₊₁ Bˣʸᵢⱼ
 * w ≥ ∑ⱼ( x̅ⱼ φʸⱼ   ) + ∑ᵢ( φˣᵢ   y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ   φʸⱼ   Bˣʸᵢⱼ )
 * w ≥ ∑ⱼ( x̅ⱼ φʸⱼ₊₁ ) + ∑ᵢ( φˣᵢ₊₁ y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ₊₁ φʸⱼ₊₁ Bˣʸᵢⱼ )
 * w ≤ ∑ⱼ( x̅ⱼ φʸⱼ   ) + ∑ᵢ( φˣᵢ₊₁ y̅ⱼ ) - ∑ᵢⱼ( φˣᵢ₊₁ φʸⱼ   Bˣʸᵢⱼ )
 * w ≤ ∑ⱼ( x̅ⱼ φʸⱼ₊₁ ) + ∑ᵢ( φˣᵢ   y̅ᵢ ) - ∑ᵢⱼ( φˣᵢ   φʸⱼ₊₁ Bˣʸᵢⱼ ).
 * ```
 * In this formulation, we introduce new continuous variables `x̅`, `y̅`, `Bˣʸ`.
 * The total number of new variables is m + n + m * n.
 *
 * In section 3.3 of Mixed-Integer Models for Nonseparable Piecewise Linear
 * Optimization: Unifying Framework and Extensions by Juan P Vielma, Shabbir
 * Ahmed and George Nemhauser, this formulation is called "Multiple Choice
 * Model".
 *
 * @note We DO NOT add the constraint
 * Bx(i) ∈ {0, 1}, By(j) ∈ {0, 1}
 * in this function. It is the user's responsibility to ensure that these binary
 * constraints are enforced. The users can also add cutting planes ∑ᵢBx(i) = 1,
 * ∑ⱼBy(j) = 1. Without these two cutting planes, (x, y, w) is still in the
 * McCormick envelope of z = x * y, but these two cutting planes "might" improve
 * the computation speed in the mixed-integer solver.
 */
void AddBilinearProductMcCormickEnvelopeMultipleChoice(
    MathematicalProgram* prog, const symbolic::Variable& x,
    const symbolic::Variable& y, const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& Bx,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& By);

}  // namespace solvers
}  // namespace drake
