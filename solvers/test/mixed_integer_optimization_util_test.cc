#include "drake/solvers/mixed_integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/gray_code.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(TestMixedIntegerUtil, TestCeilLog2) {
  // Check that CeilLog2(j) returns i + 1 for all j = 2ⁱ + 1, 2ⁱ + 2, ... , 2ⁱ⁺¹
  const int kMaxExponent = 15;
  EXPECT_EQ(0, CeilLog2(1));
  for (int i = 0; i < kMaxExponent; ++i) {
    for (int j = (1 << i) + 1; j <= (1 << (i + 1)); ++j) {
      EXPECT_EQ(i + 1, CeilLog2(j));
    }
  }
}

GTEST_TEST(TestLogarithmicSos2, TestAddSos2) {
  MathematicalProgram prog;
  auto lambda1 = prog.NewContinuousVariables(3, "lambda1");
  auto y1 =
      AddLogarithmicSos2Constraint(&prog, lambda1.cast<symbolic::Expression>());
  static_assert(std::is_same<decltype(y1), VectorXDecisionVariable>::value,
                "y1 should be a dynamic-sized vector.");
  auto lambda2 = prog.NewContinuousVariables<3>("lambda2");
  auto y2 =
      AddLogarithmicSos2Constraint(&prog, lambda2.cast<symbolic::Expression>());
  static_assert(std::is_same<decltype(y2), VectorDecisionVariable<1>>::value,
                "y2 should be a static-sized vector.");
}

void LogarithmicSos2Test(int num_lambda, bool logarithmic_binning) {
  // Solve the program
  // min λᵀ * λ
  // s.t sum λ = 1
  //     λ in sos2
  // We loop over i such that only λ(i) and λ(i+1) can be strictly positive.
  // The optimal cost is λ(i) = λ(i + 1) = 0.5.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables(num_lambda, "lambda");
  prog.AddCost(lambda.cast<symbolic::Expression>().dot(lambda));
  VectorXDecisionVariable y;
  if (logarithmic_binning) {
    y = AddLogarithmicSos2Constraint(&prog,
                                     lambda.cast<symbolic::Expression>());
  } else {
    y = prog.NewBinaryVariables(num_lambda - 1);
    AddSos2Constraint(&prog, lambda.cast<symbolic::Expression>(),
                      y.cast<symbolic::Expression>());
  }
  int num_binary_vars = y.rows();
  int num_intervals = num_lambda - 1;
  auto y_assignment = prog.AddBoundingBoxConstraint(0, 1, y);

  // If we use logarithmic binning, we will assign the binary variables y with
  // value i, expressed in Gray code.
  const auto gray_codes = math::CalculateReflectedGrayCodes(num_binary_vars);
  Eigen::VectorXd y_val(y.rows());
  for (int i = 0; i < num_intervals; ++i) {
    y_val.setZero();
    if (logarithmic_binning) {
      for (int j = 0; j < num_binary_vars; ++j) {
        y_val(j) = gray_codes(i, j);
      }
    } else {
      y_val(i) = 1;
    }
    y_assignment.constraint()->UpdateLowerBound(y_val);
    y_assignment.constraint()->UpdateUpperBound(y_val);

    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      const auto lambda_val = prog.GetSolution(lambda);
      Eigen::VectorXd lambda_val_expected = Eigen::VectorXd::Zero(num_lambda);
      lambda_val_expected(i) = 0.5;
      lambda_val_expected(i + 1) = 0.5;
      EXPECT_TRUE(CompareMatrices(lambda_val, lambda_val_expected, 1E-5,
                                  MatrixCompareType::absolute));
    }
  }
}

GTEST_TEST(TestLogarithmicSos2, Test4Lambda) {
  LogarithmicSos2Test(4, true);
}

GTEST_TEST(TestLogarithmicSos2, Test5Lambda) {
  LogarithmicSos2Test(5, true);
}

GTEST_TEST(TestLogarithmicSos2, Test6Lambda) {
  LogarithmicSos2Test(6, true);
}

GTEST_TEST(TestLogarithmicSos2, Test7Lambda) {
  LogarithmicSos2Test(7, true);
}

GTEST_TEST(TestLogarithmicSos2, Test8Lambda) {
  LogarithmicSos2Test(8, true);
}

GTEST_TEST(TestSos2, Test4Lambda) {
  LogarithmicSos2Test(4, false);
}

GTEST_TEST(TestSos2, Test5Lambda) {
  LogarithmicSos2Test(5, false);
}

GTEST_TEST(TestSos2, Test6Lambda) {
  LogarithmicSos2Test(6, false);
}

GTEST_TEST(TestSos2, Test7Lambda) {
  LogarithmicSos2Test(7, false);
}

GTEST_TEST(TestSos2, Test8Lambda) {
  LogarithmicSos2Test(8, false);
}

void LogarithmicSos1Test(int num_lambda,
                         const Eigen::Ref<const Eigen::MatrixXi> &codes) {
  // Check if we impose the constraint
  // λ is in sos1
  // and assign values to the binary variables,
  // whether the corresponding λ(i) is 1.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables(num_lambda);
  int num_digits = CeilLog2(num_lambda);
  auto y = prog.NewBinaryVariables(num_digits);
  AddLogarithmicSos1Constraint(&prog, lambda.cast<symbolic::Expression>(), y,
                               codes);
  auto binary_assignment = prog.AddBoundingBoxConstraint(0, 0, y);
  for (int i = 0; i < num_lambda; ++i) {
    Eigen::VectorXd code = codes.row(i).cast<double>().transpose();
    binary_assignment.constraint()->UpdateLowerBound(code);
    binary_assignment.constraint()->UpdateUpperBound(code);
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      Eigen::VectorXd lambda_expected(num_lambda);
      lambda_expected.setZero();
      lambda_expected(i) = 1;
      EXPECT_TRUE(CompareMatrices(prog.GetSolution(lambda), lambda_expected,
                                  1E-6, MatrixCompareType::absolute));
    }
  }
}

GTEST_TEST(TestLogarithmicSos1, Test2Lambda) {
  Eigen::Matrix<int, 2, 1> codes;
  codes << 0, 1;
  LogarithmicSos1Test(2, codes);
  // Test a different codes
  codes << 1, 0;
  LogarithmicSos1Test(2, codes);
}

GTEST_TEST(TestLogarithmicSos1, Test3Lambda) {
  Eigen::Matrix<int, 3, 2> codes;
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 0;
  // clang-format on
  LogarithmicSos1Test(3, codes);
  // Test a different codes
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 1;
  // clang-format on
  LogarithmicSos1Test(3, codes);
}

GTEST_TEST(TestLogarithmicSos1, Test4Lambda) {
  Eigen::Matrix<int, 4, 2> codes;
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 0,
           1, 1;
  // clang-format on
  LogarithmicSos1Test(4, codes);
  // Test a different codes
  // clang-format off
  codes << 0, 0,
           0, 1,
           1, 1,
           1, 0;
  // clang-format on
  LogarithmicSos1Test(4, codes);
}

GTEST_TEST(TestLogarithmicSos1, Test5Lambda) {
  auto codes = math::CalculateReflectedGrayCodes<3>();
  LogarithmicSos1Test(5, codes.topRows<5>());
}

GTEST_TEST(TestBilinearProductMcCormickEnvelopeSos2, AddConstraint) {
  // Test if the return argument from
  // AddBilinearProductMcCormickEnvelopeSos2 has the right type.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>()(0);
  auto y = prog.NewContinuousVariables<1>()(0);
  auto w = prog.NewContinuousVariables<1>()(0);

  const Eigen::Vector3d phi_x_static(0, 1, 2);
  Eigen::VectorXd phi_x_dynamic = Eigen::VectorXd::LinSpaced(3, 0, 2);
  const Eigen::Vector4d phi_y_static = Eigen::Vector4d::LinSpaced(0, 3);
  Eigen::VectorXd phi_y_dynamic = Eigen::VectorXd::LinSpaced(4, 0, 3);
  Vector1<symbolic::Expression> Bx = prog.NewBinaryVariables<1>();
  Vector2<symbolic::Expression> By = prog.NewBinaryVariables<2>();
  auto lambda1 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_static, phi_y_static, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(
      std::is_same<decltype(lambda1), MatrixDecisionVariable<3, 4>>::value,
      "lambda should be a static matrix");

  auto lambda2 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_dynamic, phi_y_static, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(std::is_same<decltype(lambda2),
                             MatrixDecisionVariable<Eigen::Dynamic, 4>>::value,
                "lambda's type is incorrect");

  auto lambda3 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_static, phi_y_dynamic, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(std::is_same<decltype(lambda3),
                             MatrixDecisionVariable<3, Eigen::Dynamic>>::value,
                "lambda's type is incorrect");

  auto lambda4 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_dynamic, phi_y_dynamic, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(
      std::is_same<
          decltype(lambda4),
          MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic>>::value,
      "lambda's type is incorrect");
}

class BilinearProductMcCormickEnvelopeSos2Test
    : public ::testing::TestWithParam<std::tuple<int, int, IntervalBinning>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BilinearProductMcCormickEnvelopeSos2Test)

  BilinearProductMcCormickEnvelopeSos2Test()
      : prog_{},
        num_interval_x_{std::get<0>(GetParam())},
        num_interval_y_{std::get<1>(GetParam())},
        binning_{std::get<2>(GetParam())},
        w_{prog_.NewContinuousVariables<1>()(0)},
        x_{prog_.NewContinuousVariables<1>()(0)},
        y_{prog_.NewContinuousVariables<1>()(0)},
        phi_x_{Eigen::VectorXd::LinSpaced(num_interval_x_ + 1, 0, 1)},
        phi_y_{Eigen::VectorXd::LinSpaced(num_interval_y_ + 1, 0, 1)},
        Bx_size_{binning_ == IntervalBinning::kLogarithmic
                     ? CeilLog2(num_interval_x_)
                     : num_interval_x_},
        By_size_{binning_ == IntervalBinning::kLogarithmic
                     ? CeilLog2(num_interval_y_)
                     : num_interval_y_},
        Bx_{prog_.NewBinaryVariables(Bx_size_)},
        By_{prog_.NewBinaryVariables(By_size_)} {}

 protected:
  MathematicalProgram prog_;
  const int num_interval_x_;
  const int num_interval_y_;
  const IntervalBinning binning_;
  const symbolic::Variable w_;
  const symbolic::Variable x_;
  const symbolic::Variable y_;
  const Eigen::VectorXd phi_x_;
  const Eigen::VectorXd phi_y_;
  const int Bx_size_;
  const int By_size_;
  const VectorXDecisionVariable Bx_;
  const VectorXDecisionVariable By_;
};

TEST_P(BilinearProductMcCormickEnvelopeSos2Test, LinearObjectiveTest) {
  // Solve the program min aᵀ * [x;y;w]
  // s.t (x, y, w) is in the convex hull of the (x, y, x*y).
  // We fix x and y to each intervals.
  // We expect the optimum obtained at one of the vertices of the tetrahedron.
  const MatrixXDecisionVariable lambda =
      AddBilinearProductMcCormickEnvelopeSos2(
          &prog_, x_, y_, w_, phi_x_, phi_y_, Bx_.cast<symbolic::Expression>(),
          By_.cast<symbolic::Expression>(), binning_);
  const Eigen::MatrixXi gray_codes_x =
      math::CalculateReflectedGrayCodes(Bx_.rows());
  const Eigen::MatrixXi gray_codes_y =
      math::CalculateReflectedGrayCodes(By_.rows());

  // We will assign the binary variables Bx_ and By_ to determine which interval
  // is active. If we use logarithmic binning, then Bx_ and By_ take values in
  // the gray code, representing integer i and j, such that x is constrained in
  // [φx(i), φx(i+1)], y is constrained in [φy(j), φy(j+1)].
  auto Bx_constraint =
      prog_.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(Bx_.rows()),
                                     Eigen::VectorXd::Zero(Bx_.rows()), Bx_);
  auto By_constraint =
      prog_.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(By_.rows()),
                                     Eigen::VectorXd::Zero(By_.rows()), By_);
  VectorDecisionVariable<3> xyw{x_, y_, w_};
  auto cost = prog_.AddLinearCost(Eigen::Vector3d::Zero(), xyw);
  Eigen::Matrix<double, 3, 8> a;
  // clang-format off
  a << 1, 1, 1, 1, -1, -1, -1, -1,
       1, 1, -1, -1, 1, 1, -1, -1,
       1, -1, 1, -1, 1, -1, 1, -1;
  // clang-format on
  for (int i = 0; i < num_interval_x_; ++i) {
    Eigen::VectorXd Bx_val(Bx_size_);
    switch (binning_) {
      case IntervalBinning::kLogarithmic :
        Bx_val = gray_codes_x.cast<double>().row(i).transpose();
        break;
      case IntervalBinning::kLinear :
        Bx_val.setZero();
        Bx_val(i) = 1;
        break;
    }

    Bx_constraint.constraint()->UpdateLowerBound(Bx_val);
    Bx_constraint.constraint()->UpdateUpperBound(Bx_val);
    for (int j = 0; j < num_interval_y_; ++j) {
      Eigen::VectorXd By_val(By_size_);
      switch (binning_) {
        case IntervalBinning::kLogarithmic :
          By_val = gray_codes_y.cast<double>().row(j).transpose();
          break;
        case IntervalBinning::kLinear :
          By_val.setZero();
          By_val(j) = 1;
          break;
      }
      By_constraint.constraint()->UpdateLowerBound(By_val);
      By_constraint.constraint()->UpdateUpperBound(By_val);

      // vertices.col(l) is the l'th vertex of the tetrahedron.
      Eigen::Matrix<double, 3, 4> vertices;
      vertices.row(0) << phi_x_(i), phi_x_(i), phi_x_(i + 1), phi_x_(i + 1);
      vertices.row(1) << phi_y_(j), phi_y_(j + 1), phi_y_(j), phi_y_(j + 1);
      vertices.row(2) = vertices.row(0).cwiseProduct(vertices.row(1));
      for (int k = 0; k < a.cols(); ++k) {
        cost.constraint()->UpdateCoefficients(a.col(k));
        GurobiSolver gurobi_solver;
        if (gurobi_solver.available()) {
          auto result = gurobi_solver.Solve(prog_);
          EXPECT_EQ(result, SolutionResult::kSolutionFound);
          Eigen::Matrix<double, 1, 4> cost_at_vertices =
              a.col(k).transpose() * vertices;
          EXPECT_NEAR(prog_.GetOptimalCost(), cost_at_vertices.minCoeff(),
                      1E-4);
          // Check that λ has the correct value, except λ(i, j), λ(i, j+1),
          // λ(i+1, j) and λ(i+1, j+1), all other entries in λ are zero.
          double w{0};
          double x{0};
          double y{0};
          for (int m = 0; m <= num_interval_x_; ++m) {
            for (int n = 0; n <= num_interval_y_; ++n) {
              if (!((m == i && n == j) || (m == i && n == (j + 1)) ||
                    (m == (i + 1) && n == j) ||
                    (m == (i + 1) && n == (j + 1)))) {
                EXPECT_NEAR(prog_.GetSolution(lambda(m, n)), 0, 1E-5);
              } else {
                double lambda_mn{prog_.GetSolution(lambda(m, n))};
                x += lambda_mn * phi_x_(m);
                y += lambda_mn * phi_y_(n);
                w += lambda_mn * phi_x_(m) * phi_y_(n);
              }
            }
          }
          EXPECT_NEAR(prog_.GetSolution(x_), x, 1E-4);
          EXPECT_NEAR(prog_.GetSolution(y_), y, 1E-4);
          EXPECT_NEAR(prog_.GetSolution(w_), w, 1E-4);
        }
      }
    }
  }
}

INSTANTIATE_TEST_CASE_P(
    TestMixedIntegerUtil, BilinearProductMcCormickEnvelopeSos2Test,
    ::testing::Combine(::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<IntervalBinning>{
                           IntervalBinning::kLogarithmic,
                           IntervalBinning::kLinear})));
}  // namespace
}  // namespace solvers
}  // namespace drake
