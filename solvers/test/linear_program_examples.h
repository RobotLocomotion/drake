#pragma once

#include <memory>
#include <optional>
#include <ostream>
#include <tuple>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/test/optimization_examples.h"

namespace drake {
namespace solvers {
namespace test {
/// Test a simple linear programming problem with zero cost, i.e. a feasibility
/// problem
///    0 <= x0 + 2x1 + 3x2 <= 10
/// -inf <=       x1 - 2x2 <= 3
///   -1 <= 0x0+ 0x1 + 0x2 <= 0
///           x1 >= 1
class LinearFeasibilityProgram : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearFeasibilityProgram)

  explicit LinearFeasibilityProgram(ConstraintForm constraint_form);

  ~LinearFeasibilityProgram() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<3> x_;
};

/// Adapt from the linear programming example
/// http://cvxopt.org/examples/tutorial/lp.html
/// Solve the following linear program
/// min     2x0 + x1 + 4
/// s.t  -inf <= -x0 + x1 <= 1
///         2 <= x0 + x1  <=inf
///      -inf <= x0 - 2x1 <= 4
///      x1 >= 2
///      x0 >= 0
/// The optimal solution is x0 = 1, x1 = 2
class LinearProgram0 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram0)

  LinearProgram0(CostForm cost_form, ConstraintForm constraint_form);

  ~LinearProgram0() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<2> x_;
  Eigen::Vector2d x_expected_;
};

// Test a simple linear programming problem with only bounding box constraint
// on x.
// min x0 - 2*x1 + 3
//     0 <= x0 <= 2
//    -1 <= x1 <= 4
// The optimal solution is (0, 4)
class LinearProgram1 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram1)

  LinearProgram1(CostForm cost_form, ConstraintForm constraint_form);

  ~LinearProgram1() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<2> x_;
  Eigen::Vector2d x_expected_;
};

// Test a simple linear programming problem
// Adapted from https://docs.mosek.com/10.0/capi/tutorial-lo-shared.html
// min -3x0 - x1 - 5x2 - x3
// s.t     3x0 +  x1 + 2x2        = 30
//   15 <= 2x0 +  x1 + 3x2 +  x3 <= inf
//  -inf<=       2x1       + 3x3 <= 25
// -inf <=  x0 + 2x1       + x3  <= inf
// -100 <=  x0       + 2x2       <= 40
//           0 <= x0 <= inf
//           0 <= x1 <= 10
//           0 <= x2 <= inf
//           0 <= x3 <= inf
// The optimal solution is at (0, 0, 15, 25/3)
class LinearProgram2 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram2)

  LinearProgram2(CostForm cost_form, ConstraintForm constraint_form);

  ~LinearProgram2() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<4> x_;
  Eigen::Vector4d x_expected_;
};

// Test a simple linear programming problem
// Adapt from http://people.brunel.ac.uk/~mastjjb/jeb/or/morelp.html
// min 4x0 + 5x1 + 6x2
// s.t.
//     x0 + x1 >= 11
//     x0 - x1 <= 5
//     x2 - x0 - x1 = 0
//     7x0 >= 35 - 12x1
//     x0 >= 0 x1 >= 0 x2 >= 0
// The optimal solution is at (8, 3, 11)
class LinearProgram3 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram3)

  LinearProgram3(CostForm cost_form, ConstraintForm constraint_form);

  ~LinearProgram3() override {};

  void CheckSolution(const MathematicalProgramResult& result) const override;

 private:
  VectorDecisionVariable<3> x_;
  Eigen::Vector3d x_expected_;
};

enum class LinearProblems {
  kLinearFeasibilityProgram = 0,
  kLinearProgram0 = 1,
  kLinearProgram1 = 2,
  kLinearProgram2 = 3,
  kLinearProgram3 = 4,
};

std::ostream& operator<<(std::ostream& os, LinearProblems value);

class LinearProgramTest
    : public ::testing::TestWithParam<
        std::tuple<CostForm, ConstraintForm, LinearProblems>> {
 public:
  LinearProgramTest();

  OptimizationProgram* prob() const {return prob_.get();}

 private:
  std::unique_ptr<OptimizationProgram> prob_;
};

std::vector<LinearProblems> linear_problems();

/**
 * An infeasible linear program.
 * max x0 + x1
 * s.t x0 + 2 * x1 <= 3;
 *     2 * x0 + x1 == 4
 *     x0 >= 0, x1 >= 2
 */
class InfeasibleLinearProgramTest0 : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InfeasibleLinearProgramTest0)

  InfeasibleLinearProgramTest0();

  ~InfeasibleLinearProgramTest0() override {}

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
};

/**
 * An unbounded linear program.
 * max x0 + x1
 * s.t 2 * x0 + x1 >= 4
 *     x0 >= 0, x1 >= 2
 */
class UnboundedLinearProgramTest0 : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnboundedLinearProgramTest0)

  UnboundedLinearProgramTest0();

  ~UnboundedLinearProgramTest0() override {}

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector2<symbolic::Variable> x_;
};

/**
 * An unbounded linear program.
 * min x0 + 2*x1 + 3*x2 + 2.5*x3 + 2
 * s.t x0 + x1 - x2 + x3 <= 3
 *     1 <= x0 + 2 * x1 - 2 * x2 + 4 * x3 <= 3
 *     0 <= x0, x2 <= 1
 */
class UnboundedLinearProgramTest1 : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnboundedLinearProgramTest1)

  UnboundedLinearProgramTest1();

  ~UnboundedLinearProgramTest1() override {}

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
};

/**
 When adding constraints and costs, we intentionally put duplicated variables in
 each constraint/cost binding, so as to test if Drake's solver wrapper can
 handle duplicated variables correctly.
 min x0 + 3 * x1 + 4 * x2 + 3
 s.t x0 + 2 * x1 + x2 <= 3
     x0 + 2 * x2 = 1
     1 <= 2 * x0 + x1 <= 3
     x0 + 2x1 + 3x2 >= 0
 */
class DuplicatedVariableLinearProgramTest1 : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DuplicatedVariableLinearProgramTest1)

  DuplicatedVariableLinearProgramTest1();

  void CheckSolution(
      const SolverInterface& solver,
      const std::optional<SolverOptions>& solver_options = std::nullopt,
      double tol = 1E-7) const;

 protected:
  std::unique_ptr<MathematicalProgram> prog_;
  Vector3<symbolic::Variable> x_;
};

/**
 * Test getting dual solution for LP.
 * This LP has inequality constraints.
 */
void TestLPDualSolution1(const SolverInterface& solver, double tol = 1e-6);

/** This LP has only bounding box constraints. */
void TestLPDualSolution2(const SolverInterface& solver, double tol = 1e-6);

/** This LP has only bounding box constraints. The decision variable is
 * scaled.*/
void TestLPDualSolution2Scaled(const SolverInterface& solver,
                               double tol = 1e-6);

/** This LP has only bounding box constraints. */
void TestLPDualSolution3(const SolverInterface& solver, double tol = 1e-6);

/** This test confirms that the solver can solve problems with poorly scaled
 * data. See github issue https://github.com/RobotLocomotion/drake/issues/15341
 * for more discussion. Mathematically this program finds the point with the
 * smallest infinity norm to (0.99, 1.99). The point is within the convex hull
 * of four points (eps, eps), (1, 1), (eps, 2), (1, 2).
 */
void TestLPPoorScaling1(
    const SolverInterface& solver, bool expect_success = true,
    double tol = 1E-12,
    const std::optional<SolverOptions>& options = std::nullopt);

/** This test confirms that the solver can solve problems with poorly scaled
 * data. See github issue https://github.com/RobotLocomotion/drake/issues/15341
 * for more discussion.
 * The optimal solution isn't computed analytically, hence we take a loose
 * tolerance.
 */
void TestLPPoorScaling2(
    const SolverInterface& solver, bool expect_success = true,
    double tol = 1E-4,
    const std::optional<SolverOptions>& options = std::nullopt);
}  // namespace test
}  // namespace solvers
}  // namespace drake
