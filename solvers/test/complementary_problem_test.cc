#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {
namespace {
// A problem from J.F. Bard, Convex two-level optimization,
// Mathematical Programming 40(1), 15-27, 1988.
// min (x-5)² + (2*y + 1)²
// s.t 2*(y-1) - 1.5*x + l(0) - 0.5*l(1) + l(2) = 0
//     0 <= l(0) ⊥ 3 * x - y - 3 >= 0
//     0 <= l(1) ⊥ -x + 0.5*y + 4 >= 0
//     0 <= l(2) ⊥ -x - y + 7 >= 0
//     x >= 0, y >= 0
GTEST_TEST(TestComplementaryProblem, bard1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  auto y = prog.NewContinuousVariables<1>();
  auto l = prog.NewContinuousVariables<3>();
  prog.AddCost(pow(x(0) - 5, 2) + pow(2 * y(0) + 1, 2));
  prog.AddLinearConstraint(
      2 * (y(0) - 1) - 1.5 * x(0) + l(0) - 0.5 * l(1) + l(2) == 0);
  // TODO(hongkai.dai): write this linear complementarity constraint in symbolic
  // form.
  // z = [x;y;l]
  // M = [3, -1, 0, 0, 0]
  //     [-1, 0.5, 0, 0, 0]
  //     [-1, -1, 0, 0, 0]
  //     [0, 0, 0, 0, 0]
  //     [0, 0, 0, 0, 0]
  // q = [-3; 4; 7; 0; 0]
  // 0 <= z ⊥ M*z + q >= 0
  Eigen::Matrix<double, 5, 5> M;
  // clang-format off
  M <<  3,  -1, 0, 0, 0,
       -1, 0.5, 0, 0, 0,
       -1,  -1, 0, 0, 0,
        0,   0, 0, 0, 0,
        0,   0, 0, 0, 0;
  // clang-format on
  Eigen::Matrix<double, 5, 1> q;
  q << -3, 4, 7, 0, 0;
  prog.AddLinearComplementarityConstraint(M, q, {x, y, l});

  SnoptSolver snopt_solver;
  if (snopt_solver.available()) {
    auto result = snopt_solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    auto x_val = result.GetSolution(x);
    auto y_val = result.GetSolution(y);
    EXPECT_NEAR(x_val(0), 1, 1E-6);
    EXPECT_NEAR(y_val(0), 0, 1E-6);
  }
}

// Problem 2 from Fukushima, M. Luo, Z.-Q.Pang, J.-S.,
// "A globally convergent Sequential Quadratic Programming
// Algorithm for Mathematical Programs with Linear Complementarity
// Constraints", Computational Optimization and Applications, 10(1),
// pp. 5-34, 1998.
// min 0.5(x(0) + x(1) + y(0) - 15)² + 0.5(x(0) + x(1) + y(1) - 15)²
// s.t 0 <= y(0) ⊥ 8/3 * x(0) + 2 * x(1) + 2 * y(0) + 8 / 3 * y(1) - 36 >= 0
//     0 <= y(1) ⊥ 2 * x(0) + 5/4 * x(1) + 5/4 * y(0) + 2 * y(1) - 25 >= 0
//     0 <= x <= 10
GTEST_TEST(TestComplementaryProblem, flp2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto y = prog.NewContinuousVariables<2>();
  prog.AddCost(0.5 * pow(x(0) + x(1) + y(0) - 15, 2) +
               0.5 * (x(0) + x(1) + y(1) - 15, 2));
  Eigen::Matrix4d M;
  // clang-format off
  M << 0, 0, 0, 0,
       0, 0, 0, 0,
       8.0 / 3, 2, 2, 8.0/3,
       2, 5.0/4, 5.0/4, 2;
  // clang-format off;
  Eigen::Vector4d q(0, 0, -36, -25);
  prog.AddLinearComplementarityConstraint(M, q, {x, y});
  prog.AddBoundingBoxConstraint(0, 10, x);
  SnoptSolver snopt_solver;
  if (snopt_solver.available()) {
    MathematicalProgramResult result = Solve(prog);
    EXPECT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(x);
    const auto y_val = result.GetSolution(y);
    // Choose 1e-6 as the precision, since that is the default minor feasibility
    // tolerance of SNOPT.
    double precision = 1E-6;
    EXPECT_TRUE((x_val.array() >= -precision).all());
    EXPECT_TRUE((x_val.array() <= 10 + precision).all());
    EXPECT_TRUE((y_val.array() >= -precision).all());
    Eigen::Vector4d z_val;
    z_val << x_val, y_val;
    const Eigen::Vector2d Mz_plus_q = (M * z_val + q).bottomRows<2>();
    EXPECT_TRUE((Mz_plus_q.array() >= -precision).all());
    EXPECT_NEAR(y_val.dot(Mz_plus_q), 0, precision);
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake
