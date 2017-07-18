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
 * Set of type 2 (sos2) constraint. The sos2 constraint says that
 * <pre>
 *   λ(0) + ... + λ(n) = 1
 *   λ(i) ≥ 0 ∀i
 *   ∃ j ∈ {0, 1, ..., n-1}, s.t λ(j) + λ(j + 1) = 1
 * </pre>
 * @tparam NumLambda The length of the lambda vector. NumLambda = n + 1.
 */
template<int NumLambda>
struct LogarithmicSOS2NewBinaryVariables {
  typedef VectorDecisionVariable<CeilLog2(NumLambda - 1)> type;
  static const int Rows = CeilLog2(NumLambda - 1);
};

template<>
struct LogarithmicSOS2NewBinaryVariables<Eigen::Dynamic> {
  typedef VectorXDecisionVariable type;
  static const int Rows = Eigen::Dynamic;
};

/**
 * Adds the special ordered set 2 (sos2) constraint,
 * <pre>
 *   λ(0) + ... + λ(n) = 1
 *   λ(i) ≥ 0 ∀i
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
 * @param prog Add the sos2 constraint to this mathematical program.
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
        std::is_same<typename Derived::Scalar, symbolic::Expression>::value,
    typename LogarithmicSOS2NewBinaryVariables<
        Derived::RowsAtCompileTime>::type>::type
AddLogarithmicSOS2Constraint(MathematicalProgram* prog, const Derived& lambda,
                             const std::string& binary_variable_name = "y") {
  int binary_variable_size = CeilLog2(lambda.rows() - 1);
  auto y = prog->NewBinaryVariables<
      LogarithmicSOS2NewBinaryVariables<Derived::RowsAtCompileTime>::Rows, 1>(
      binary_variable_size, 1, binary_variable_name);
  AddLogarithmicSOS2Constraint(prog, lambda, y);
  return y;
}

/** Adds the special ordered set 2 (sos2) constraint,
 * @see AddLogarithmicSOS2Constraint.
 */
void AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y);

/**
 * Adds the special ordered set of type 1 (sos1) constraint. Namely
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
 * @param prog The program to which the sos1 constraint is added.
 * @param lambda lambda is in sos1.
 * @param y The binary variables indicating which λ is positive. For a given
 * assignment on the binary variable `y`, if (y(0), ..., y(⌈log₂(n)⌉) represents
 * integer M in `codes`, then only λ(M) is positive. Namely, if
 * (y(0), ..., y(⌈log₂(n)⌉) equals to codes.row(M), then λ(M) = 1
 * @param codes A n x ⌈log₂(n)⌉ matrix. code.row(i) represents integer i.
 * No two rows of `codes` can be the same. @throws std::runtime_error if
 * @p codes has a non-binary entry (0, 1).
 */
void AddLogarithmicSOS1Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::MatrixXi>& codes);

/**
 * Constrain `w` to approximate the bilinear product x * y. We know
 * that x is in one of the intervals [φx(i), φx(i+1)], y is in one of the
 * intervals [φy(j), φy(j+1)]. The variable `w` is constrained to be in the
 * convex hull of x * y for x in [φx(i), φx(i+1)], y in [φy(j), φy(j+1)], namely
 * (x, y, w) is in the tetrahedron, with vertices [φx(i), φy(j), φx(i)*φy(j)],
 * [φx(i+1), φy(j), φx(i+1)*φy(j)], [φx(i), φy(j+1), φx(i)*φy(j+1)] and
 * [φx(i+1), φy(j+1), φx(i+1)*φy(j+1)]
 * @param prog The program to which the bilinear product constraint is added
 * @param x The decision variable.
 * @param y The decision variable.
 * @param w The expression to approximate x * y
 * @param phi_x The end points of the intervals for `x`.
 * @param phi_y The end points of the intervals for `y`.
 * @param Bx The binary variable, to determine in which interval `x` stays.
 * If Bx represents integer M in Gray code, then `x` is in the interval
 * [φx(M), φx(M+1)].
 * @param By The binary variable, to determine in which interval `y` stays.
 * If Bx represents integer M in Gray code, then `y` is in the interval
 * [φy(M), φy(M+1)].
 */
void AddBilinearProductMcCormickEnvelopeSOS2(
    MathematicalProgram* prog,
    const symbolic::Variable& x,
    const symbolic::Variable& y,
    const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorXDecisionVariable>& Bx,
    const Eigen::Ref<const VectorXDecisionVariable>& By);
}  // namespace solvers
}  // namespace drake
