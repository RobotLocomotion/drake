#pragma once

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {

enum class EllipsoidsSeparationProblem {
  kProblem0,
  kProblem1,
  kProblem2,
  kProblem3
};

std::vector<EllipsoidsSeparationProblem> GetEllipsoidsSeparationProblems();

/// This test is taken from the course notes of EE127A from University of
/// California, Berkeley
/// The goal is to find a hyperplane, that separates two ellipsoids
/// E₁ = x₁ + R₁ * u₁, |u₁| ≤ 1
/// E₂ = x₂ + R₂ * u₂, |u₂| ≤ 1
/// A hyperplane aᵀ * x = b separates these two ellipsoids, if and only if for
/// SOCP p* = min t₁ + t₂
///           s.t t₁ ≥ |R₁ᵀ * a|
///               t₂ ≥ |R₂ᵀ * a|
///               aᵀ(x₂-x₁) = 1
/// the optimal solution p* is no larger than 1. In that case, an appropriate
/// value of b is b = 0.5(b₁ + b₂), where
/// b₁ = aᵀx₁ + |R₁ᵀ * a|
/// b₂ = aᵀx₂ - |R₂ᵀ * a|
/// @param x1  the center of ellipsoid 1
/// @param x2  the center of ellipsoid 2
/// @param R1  the shape of ellipsoid 1
/// @param R2  the shape of ellipsoid 2
class TestEllipsoidsSeparation
    : public ::testing::TestWithParam<EllipsoidsSeparationProblem> {
 public:
  TestEllipsoidsSeparation();

  void SolveAndCheckSolution(const SolverInterface& solver,
                             double tol = 1E-8);

 private:
  Eigen::VectorXd x1_;
  Eigen::VectorXd x2_;
  Eigen::MatrixXd R1_;
  Eigen::MatrixXd R2_;
  MathematicalProgram prog_;
  VectorDecisionVariable<2> t_;
  VectorXDecisionVariable a_;
};

enum class QPasSOCPProblem { kProblem0, kProblem1 };

std::vector<QPasSOCPProblem> GetQPasSOCPProblems();

/// This example is taken from the course notes of EE127A from University of
/// California, Berkeley
/// For a quadratic program
/// 0.5 * x' * Q * x + c' * x
/// s.t b_lb <= A * x <= b_ub
/// It can be casted as an SOCP, as follows
/// By introducing a new variable w = Q^{1/2}*x and y, z
/// The equivalent SOCP is
/// min c'x + y
/// s.t 2 * y >= w' * w
///     w = Q^{1/2} * x
///     b_lb <= A * x <= b_ub
/// @param Q A positive definite matrix
/// @param c A column vector
/// @param A A matrix
/// @param b_lb A column vector
/// @param b_ub A column vector
class TestQPasSOCP : public ::testing::TestWithParam<QPasSOCPProblem> {
 public:
  TestQPasSOCP();

  void SolveAndCheckSolution(const SolverInterface& solver,
                             double tol = 1E-6);

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd c_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_lb_;
  Eigen::VectorXd b_ub_;
  MathematicalProgram prog_socp_;
  MathematicalProgram prog_qp_;
  VectorXDecisionVariable x_socp_;
  symbolic::Variable y_;
  VectorXDecisionVariable x_qp_;
};

/// This example is taken from the paper
/// Applications of second-order cone programming
/// By M.S.Lobo, L.Vandenberghe, S.Boyd and H.Lebret,
/// Section 3.6
/// http://www.seas.ucla.edu/~vandenbe/publications/socp.pdf
/// The problem tries to find the equilibrium state of a mechanical
/// system, which consists of n nodes at position (x₁,y₁), (x₂,y₂), ...,
/// (xₙ,yₙ) in ℝ².
/// The nodes are connected by springs with given coefficient.
/// The spring generate force when it is stretched,
/// but not when it is compressed.
/// Namely, the spring force is
/// (spring_length - spring_rest_length) * spring_stiffness,
/// if spring_length ≥ spring_rest_length;
/// otherwise the spring force is zero.
/// weightᵢ is the mass * gravity_acceleration
/// of the i'th link.
/// The equilibrium point is obtained when the total energy is minimized
/// namely min ∑ᵢ weightᵢ * yᵢ + stiffness/2 * tᵢ²
///        s.t  sqrt((xᵢ - xᵢ₊₁)² + (yᵢ - yᵢ₊₁)²) - spring_rest_length ≤ tᵢ
///             0 ≤ tᵢ
///             (x₁,y₁) = end_pos1
///             (xₙ,yₙ) = end_pos2
/// By introducing a slack variable z ≥ t₁² + ... + tₙ₋₁², the problem
/// becomes
/// an SOCP, with both Lorentz cone and rotated Lorentz cone constraints
enum class FindSpringEquilibriumProblem { kProblem0 };
std::vector<FindSpringEquilibriumProblem> GetFindSpringEquilibriumProblems();
class TestFindSpringEquilibrium
    : public ::testing::TestWithParam<FindSpringEquilibriumProblem> {
 public:
  TestFindSpringEquilibrium();

  void SolveAndCheckSolution(const SolverInterface& solver,
                             double tol = 2E-3);

 private:
  Eigen::VectorXd weight_;
  double spring_rest_length_;
  double spring_stiffness_;
  Eigen::Vector2d end_pos1_;
  Eigen::Vector2d end_pos2_;
  MathematicalProgram prog_;
  VectorXDecisionVariable x_;
  VectorXDecisionVariable y_;
  VectorXDecisionVariable t_;
  symbolic::Variable z_;
};

/**
 * Solve the following trivial problem
 * max (2x+3)*(3x+2)
 * s.t 2x+3 >= 0
 *     3x+2 >= 0
 *     x<= 10
 * We formulate this problem as a convex second order cone constraint, by
 * regarding the cost as the square of geometric mean between 2x+3 and 3x+2.
 * The optimal is x* = 10.
 */
class MaximizeGeometricMeanTrivialProblem1 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaximizeGeometricMeanTrivialProblem1)

  MaximizeGeometricMeanTrivialProblem1();

  const MathematicalProgram& prog() const { return *prog_; }

  void CheckSolution(const MathematicalProgramResult& result, double tol);

 private:
  std::unique_ptr<MathematicalProgram> prog_;
  symbolic::Variable x_;
};

/**
 * Solve the following trivial problem
 * max (2x+3)*(3x+2)*(4x+5)
 * s.t 2x+3 >= 0
 *     3x+2 >= 0
 *     4x+5 >= 0
 *     x<= 10
 * We formulate this problem as a convex second order cone constraint, by
 * regarding the cost as the square of geometric mean between 2x+3, 3x+2 and
 * 4x+5. The optimal is x* = 10.
 */
class MaximizeGeometricMeanTrivialProblem2 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaximizeGeometricMeanTrivialProblem2)

  MaximizeGeometricMeanTrivialProblem2();

  const MathematicalProgram& prog() const { return *prog_; }

  void CheckSolution(const MathematicalProgramResult& result, double tol);

 private:
  std::unique_ptr<MathematicalProgram> prog_;
  symbolic::Variable x_;
};

/**
 * Tests maximizing geometric mean through second order cone constraint.
 * Given some points p₁, p₂, ..., pₖ ∈ ℝⁿ, find the smallest ellipsoid (centered
 * at the origin, and aligned with the axes) such that the ellipsoid contains
 * all these points.
 * This problem can be formulated as
 * max a(0) * a(1) * ... * a(n-1)
 * s.t pᵢᵀ diag(a) * pᵢ ≤ 1 for all i.
 *     a(j) > 0
 */
class SmallestEllipsoidCoveringProblem {
 public:
  // p.col(i) is the point pᵢ.
  explicit SmallestEllipsoidCoveringProblem(
      const Eigen::Ref<const Eigen::MatrixXd>& p);

  virtual ~SmallestEllipsoidCoveringProblem() {}

  const MathematicalProgram& prog() const { return *prog_; }

  void CheckSolution(const MathematicalProgramResult& result, double tol) const;

 protected:
  const VectorX<symbolic::Variable>& a() const { return a_; }

 private:
  // CheckSolution() already checks if the result is succesful, and if all the
  // points are within the ellipsoid, with at least one point on the boundary
  // of the ellipsoid. CheckSolutionExtra can do extra checks for each specific
  // problem.
  virtual void CheckSolutionExtra(const MathematicalProgramResult&,
                                  double) const {}
  std::unique_ptr<MathematicalProgram> prog_;
  VectorX<symbolic::Variable> a_;
  Eigen::MatrixXd p_;
};

class SmallestEllipsoidCoveringProblem1
    : public SmallestEllipsoidCoveringProblem {
 public:
  SmallestEllipsoidCoveringProblem1();

  ~SmallestEllipsoidCoveringProblem1() override {}

 private:
  void CheckSolutionExtra(const MathematicalProgramResult& result,
                          double tol) const override;
};

void SolveAndCheckSmallestEllipsoidCoveringProblems(
    const SolverInterface& solver, double tol);

}  // namespace test
}  // namespace solvers
}  // namespace drake
