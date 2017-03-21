#pragma once

#include <memory>
#include <tuple>
#include <vector>

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

  explicit LinearFeasibilityProgram(ConstraintForm cnstr_form);

  ~LinearFeasibilityProgram() override {};

  void CheckSolution(SolverType solver_type) const override;

 private:
  VectorDecisionVariable<3> x_;
};

/// Adapt from the linear programming example
/// http://cvxopt.org/examples/tutorial/lp.html
/// Solve the following linear program
/// min     2x0 + x1
/// s.t  -inf <= -x0 + x1 <= 1
///         2 <= x0 + x1  <=inf
///      -inf <= x0 - 2x1 <= 4
///      x1 >= 2
///      x0 >= 0
/// The optimal solution is x0 = 1, x1 = 2
class LinearProgram0 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram0)

  LinearProgram0(CostForm cost_form, ConstraintForm cnstr_form);

  ~LinearProgram0() override {};

  void CheckSolution(SolverType solver_type) const override;

 private:
  VectorDecisionVariable<2> x_;
  Eigen::Vector2d x_expected_;
};

// Test a simple linear programming problem with only bounding box constraint
// on x.
// min x0 - 2*x1
//     0 <= x0 <= 2
//    -1 <= x1 <= 4
// The optimal solution is (0, 4)
class LinearProgram1 : public OptimizationProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearProgram1)

  LinearProgram1(CostForm cost_form, ConstraintForm cnstr_form);

  ~LinearProgram1() override {};

  void CheckSolution(SolverType solver_type) const override;

 private:
  VectorDecisionVariable<2> x_;
  Eigen::Vector2d x_expected_;
};

// Test a simple linear programming problem
// Adapt from http://docs.mosek.com/7.1/capi/Linear_optimization.html
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

  LinearProgram2(CostForm cost_form, ConstraintForm cnstr_form);

  ~LinearProgram2() override {};

  void CheckSolution(SolverType solver_type) const override;

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

  LinearProgram3(CostForm cost_form, ConstraintForm cnstr_form);

  ~LinearProgram3() override {};

  void CheckSolution(SolverType solver_type) const override;

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
};
}  // namespace test
}  // namespace solvers
}  // namespace drake
