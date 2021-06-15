#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {
enum class QuadraticProblems {
  kQuadraticProgram0 = 0,
  kQuadraticProgram1 = 1,
  kQuadraticProgram2 = 2,
  kQuadraticProgram3 = 3,
  kQuadraticProgram4 = 4,
};

class QuadraticProgramTest
    : public ::testing::TestWithParam<
        std::tuple<CostForm, ConstraintForm, QuadraticProblems>> {
 public:
  QuadraticProgramTest();

  OptimizationProgram* prob() const {return prob_.get();}

 private:
  std::unique_ptr<OptimizationProgram> prob_;
};

std::vector<QuadraticProblems> quadratic_problems();

// Test a simple Quadratic Program.
// The example is taken from
// http://cvxopt.org/examples/tutorial/qp.html
// min 2x1^2 + x2^2 + x1x2 + x1 + x2
// s.t x1 >= 0
//     x2 >= 0
//     x1 + x2 = 1
class QuadraticProgram0 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticProgram0)

  QuadraticProgram0(CostForm cost_form, ConstraintForm constraint_form);

  ~QuadraticProgram0() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<2> x_;
  Eigen::Vector2d x_expected_;
};

// Adapted from the simple test on the Gurobi documentation.
//  min    x^2 + x*y + y^2 + y*z + z^2 + 2 x
//  subj to 4 <=   x + 2 y + 3 z <= inf
//       -inf <=  -x -   y       <= -1
//        -20 <=         y + 2 z <= 100
//       -inf <=   x +   y + 2 z <= inf
//               3 x +   y + 3 z  = 3
//                 x, y, z >= 0
//   The optimal solution is (0, 1, 2/3)
class QuadraticProgram1 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticProgram1)

  QuadraticProgram1(CostForm cost_form, ConstraintForm constraint_form);

  ~QuadraticProgram1() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<3> x_;
  Eigen::Vector3d x_expected_;
};

// Closed form (exact) solution test of QP problem.
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
// The test also verifies the quadratic program works when
// matrix Q has off-diagonal terms.
class QuadraticProgram2 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticProgram2)

  QuadraticProgram2(CostForm cost_form, ConstraintForm constraint_form);

  ~QuadraticProgram2() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<5> x_;
  Eigen::Matrix<double, 5, 1> x_expected_;
};

// Closed form (exact) solution test of QP problem.
// Added as multiple QP cost terms
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
// We impose the cost
//   0.5 * x.head<4>()'*Q1 * x.head<4>() + b1'*x.head<4>()
// + 0.5 * x.tail<4>()'*Q2 * x.tail<4>() + b2'*x.tail<4>()
// This is to test that we can add multiple costs for the same variables (in
// this case, the quadratic costs on x(2), x(3) are added for twice).
class QuadraticProgram3 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticProgram3);

  QuadraticProgram3(CostForm cost_form, ConstraintForm constraint_form);

  ~QuadraticProgram3() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<6> x_;
  Vector6d x_expected_;
};

// Test the simple QP
// min x(0)^2 + x(1)^2 + 2 * x(2)^2
// s.t x(0) +   x(1) = 1
//     x(0) + 2*x(2) = 2
// The optimal solution should be
// x(0) = 4/5, x(1) = 1/5, x(2) = 3/5
class QuadraticProgram4 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuadraticProgram4)

  QuadraticProgram4(CostForm cost_form, ConstraintForm constraint_form);

  ~QuadraticProgram4() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<3> x_;
  Eigen::Vector3d x_expected_;
};

// Solve a series of QPs with the objective being the Euclidean distance
// from a desired point which moves along the unit circle (L2 ball), to a point
// constrained to lie inside the L1 ball.  Implemented in 2D, so that the
// active set moves along 4 faces of the L1 ball.
void TestQPonUnitBallExample(const SolverInterface& solver);

/**
 * Test getting the dual solution for a QP problem.
 * This QP problem has active linear equality constraints.
 */
void TestQPDualSolution1(const SolverInterface& solver,
                         const SolverOptions& solver_options = {},
                         double tol = 1e-6);

/**
 * Test getting the dual solution for a QP problem.
 * This QP problem has active linear inequality constraints.
 */
void TestQPDualSolution2(const SolverInterface& solver);

/**
 * Test getting the dual solution for a QP problem.
 * This QP problem has active bounding box constraints.
 * @param tol The tolerance on checking the solution.
 * @param sensitivity_tol The tolerance on checking the constraint sensitivity.
 */
void TestQPDualSolution3(const SolverInterface& solver, double tol = 1e-6,
                         double sensitivity_tol = 2e-5);

/**
 * Test getting the dual solution for an equality constrained QP.
 */
void TestEqualityConstrainedQPDualSolution1(const SolverInterface& solver);

/**
 * Test getting the dual solution for an equality constrained QP.
 */
void TestEqualityConstrainedQPDualSolution2(const SolverInterface& solver);

/**
 * Test nonconvex QP.
 */
void TestNonconvexQP(const SolverInterface& solver, bool convex_solver,
                     double tol = 1E-5);
}  // namespace test
}  // namespace solvers
}  // namespace drake
