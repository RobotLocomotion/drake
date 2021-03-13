#include <limits>

#include "ClpSimplex.hpp"
#include "CoinBuild.hpp"
#include <gtest/gtest.h>

// TODO(hongkai.dai): remove this test file when we add ClpSolver class to
// Drake.
const double kInf = std::numeric_limits<double>::infinity();
GTEST_TEST(ClpTest, Test1) {
  ClpSimplex model;
  // Solve a simple LP problem
  // min x1 + 2*x2 + 4
  // 1 <= x1 <= 4
  // -2 <= x2 <= 3
  // The optimal solution is (1, -2)
  double objective[] = {1., 2.};
  double x_lo[] = {1., -2.};
  double x_up[] = {4., 3.};
  model.loadProblem(2, 0, nullptr, nullptr, nullptr, x_lo, x_up, objective,
                    nullptr, nullptr);
  model.setObjectiveOffset(-4.);
  model.primal();
  const double tol = 1E-8;
  EXPECT_EQ(model.status(), 0);
  EXPECT_NEAR(model.getObjValue(), 1., tol);
  EXPECT_NEAR(*model.getColSolution(), 1., tol);
  EXPECT_NEAR(*(model.getColSolution() + 1), -2., tol);
}

GTEST_TEST(ClpTest, Test2) {
  ClpSimplex model;
  // min     2x0 + x1
  // s.t  -inf <= -x0 + x1 <= 1
  //         2 <= x0 + x1  <=inf
  //      -inf <= x0 - 2x1 <= 4
  //      x1 >= 2
  //      x0 >= 0
  // The optimal solution is x0 = 1, x1 = 2
  model.resize(0, 2);
  model.setColumnLower(0, 0);
  model.setColumnLower(1, 2);
  model.setObjCoeff(0, 2.);
  model.setObjCoeff(1, 1.);

  CoinBuild build_object;
  int column_indices[] = {0, 1};
  double column_coeff[] = {-1., 1.};
  build_object.addRow(2, column_indices, column_coeff, -kInf, 1.);
  column_coeff[0] = 1.;
  build_object.addRow(2, column_indices, column_coeff, 2., kInf);
  column_coeff[1] = -2;
  build_object.addRow(2, column_indices, column_coeff, -kInf, 4);
  model.addRows(build_object);
  model.primal();
  const double tol = 1E-8;
  EXPECT_EQ(model.status(), 0);
  EXPECT_NEAR(model.getObjValue(), 4., tol);
  EXPECT_NEAR(*model.getColSolution(), 1., tol);
  EXPECT_NEAR(*(model.getColSolution() + 1), 2., tol);
}
