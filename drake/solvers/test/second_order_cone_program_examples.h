#pragma once

#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"

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

/// This test is taken from
/// https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_ell_sep.html
/// The goal is to find a hyperplane, that separates two ellipsoids
/// E1 = x1 + R1 * u1, u1' * u1<=1
/// E2 = x2 + R2 * u2, u2' * u2<=1
/// A hyperplane a' * x = b separates these two ellipsoids, if and only if for
/// SOCP p* = min t1 + t2
///           s.t t1 >= |R1' * a|
///               t2 >= |R2' * a|
///               a'*(x2-x1) = 1
/// the optimal solution p* is no larger than 1. In that case, an approppriate
/// value of b is b = 0.5 * (b1 + b2), where
/// b1 = a' * x1 + |R1' * a|
/// b2 = a' * x2 - |R2' * a|
/// @param x1  the center of ellipsoid 1
/// @param x2  the center of ellipsoid 2
/// @param R1  the shape of ellipsoid 1
/// @param R2  the shape of ellipsoid 2
class TestEllipsoidsSeparation
    : public ::testing::TestWithParam<EllipsoidsSeparationProblem> {
 public:
  TestEllipsoidsSeparation();

  void SolveAndCheckSolution(const MathematicalProgramSolverInterface& solver);

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

/// This example is taken from
/// https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_qp_as_socp.html
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

  void SolveAndCheckSolution(const MathematicalProgramSolverInterface& solver);

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
/// system, which consists of N nodes at position (x1,y1), (x2,y2), ..., (xN,yN)
/// in R2.
/// The nodes are connected by springs with given coefficient.
/// The spring generate force when it is stretched,
/// but not when it is compressed.
/// Namely, the spring force is
/// (spring_length - spring_rest_length) * spring_stiffness,
/// if spring_length >= spring_rest_length;
/// otherwise the spring force is zero.
/// weight_i is the mass * gravity_acceleration
/// of the i'th link.
/// The equilibrium point is obtained when the total energy is minimized
/// namely min sum_i weight_i * yi + stiffness/2 * t_i^2
///        s.t  sqrt((x(i) - x(i+1))^2 + (y(i) - y(i+1))^2) - spring_rest_length
/// <= t_i
///             0 <= t_i
///             (x1,y1) = end_pos1
///             (xN,yN) = end_pos2
/// By introducing a slack variable z >= t_1^2 + ... + t_(N-1)^2, the problem
/// becomes
/// an SOCP, with both Lorentz cone and rotated Lorentz cone constraints
enum class FindSpringEquilibriumProblem { kProblem0 };
std::vector<FindSpringEquilibriumProblem> GetFindSpringEquilibriumProblems();
class TestFindSpringEquilibrium
    : public ::testing::TestWithParam<FindSpringEquilibriumProblem> {
 public:
  TestFindSpringEquilibrium();

  void SolveAndCheckSolution(const MathematicalProgramSolverInterface& solver,
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
}  // namespace test
}  // namespace solvers
}  // namespace drake