#include "drake/solvers/mixed_integer_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
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

void LogarithmicSOS2Test(int num_lambda) {
  // Solve the program
  // min λᵀ * λ
  // s.t sum λ = 1
  //     λ in sos2
  // We loop over i such that only λ(i) and λ(i+1) can be strictly positive.
  // The optimal cost is λ(i) = λ(i + 1) = 0.5.
  MathematicalProgram prog;
  auto lambda = prog.NewContinuousVariables(num_lambda, "lambda");
  prog.AddCost(lambda.cast<symbolic::Expression>().dot(lambda));
  auto y =
      AddLogarithmicSOS2Constraint(&prog, lambda.cast<symbolic::Expression>());
  int num_binary_vars = y.rows();
  int num_intervals = num_lambda - 1;
  auto y_assignment = prog.AddBoundingBoxConstraint(0, 1, y);

  // We assign the binary variables y with value i, expressed in Gray code.
  const auto gray_codes = math::CalculateReflectedGrayCodes(num_binary_vars);
  Eigen::VectorXd y_val(num_binary_vars);
  for (int i = 0; i < num_intervals; ++i) {
    y_val.setZero();
    for (int j = 0; j < num_binary_vars; ++j) {
      y_val(j) = gray_codes(i, j);
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

GTEST_TEST(TestLogarithmicSOS2, Test4Lambda) {
  LogarithmicSOS2Test(4);
}

GTEST_TEST(TestLogarithmicSOS2, Test5Lambda) {
  LogarithmicSOS2Test(5);
}

GTEST_TEST(TestLogarithmicSOS2, Test6Lambda) {
  LogarithmicSOS2Test(6);
}

GTEST_TEST(TestLogarithmicSOS2, Test7Lambda) {
  LogarithmicSOS2Test(7);
}

GTEST_TEST(TestLogarithmicSOS2, Test8Lambda) {
  LogarithmicSOS2Test(8);
}

class BilinearProductMcCormickEnvelopeSOS2Test : public ::testing::TestWithParam<std::tuple<int, int>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BilinearProductMcCormickEnvelopeSOS2Test)

  BilinearProductMcCormickEnvelopeSOS2Test()
      : prog_{},
        num_interval_x_{std::get<0>(GetParam())},
        num_interval_y_{std::get<1>(GetParam())},
        w_{prog_.NewContinuousVariables<1>()(0)},
        x_{prog_.NewContinuousVariables<1>()(0)},
        y_{prog_.NewContinuousVariables<1>()(0)},
        phi_x_{Eigen::VectorXd::LinSpaced(num_interval_x_ + 1, 0, 1)},
        phi_y_{Eigen::VectorXd::LinSpaced(num_interval_y_ + 1, 0, 1)},
        Bx_{prog_.NewBinaryVariables(CeilLog2(num_interval_x_))},
        By_{prog_.NewBinaryVariables(CeilLog2(num_interval_y_))} {
    AddBilinearProductMcCormickEnvelopeSOS2(&prog_, x_, y_, w_, phi_x_, phi_y_, Bx_, By_);
  }

 protected:
  MathematicalProgram prog_;
  int num_interval_x_;
  int num_interval_y_;
  symbolic::Variable w_;
  symbolic::Variable x_;
  symbolic::Variable y_;
  Eigen::VectorXd phi_x_;
  Eigen::VectorXd phi_y_;
  VectorXDecisionVariable Bx_;
  VectorXDecisionVariable By_;
};

TEST_P(BilinearProductMcCormickEnvelopeSOS2Test, LinearObjectiveTest) {
  // Solve the program min aᵀ * [x;y;w]
  // s.t (x, y, w) is in the convex hull of the (x, y, x*y).
  // We fix x and y to each intervals.
  Eigen::MatrixXi gray_codes_x = math::CalculateReflectedGrayCodes(Bx_.rows());
  Eigen::MatrixXi gray_codes_y = math::CalculateReflectedGrayCodes(By_.rows());
  auto x_gray_code_cnstr = prog_.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(Bx_.rows()), Eigen::VectorXd::Zero(Bx_.rows()), Bx_);
  auto y_gray_code_cnstr = prog_.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(By_.rows()), Eigen::VectorXd::Zero(By_.rows()), By_);
  VectorDecisionVariable<3> xyw{x_, y_, w_};
  auto cost = prog_.AddLinearCost(Eigen::Vector3d::Zero(), xyw);
  Eigen::Matrix<double, 3, 4> a;
  a << 1, 0, 1, -1,
       -1, 1, 2, 1,
       2, -1, -2, 3;
  for (int i = 0; i < num_interval_x_; ++i) {
    Eigen::VectorXd x_gray_code = gray_codes_x.cast<double>().row(i).transpose();
    x_gray_code_cnstr.constraint()->UpdateLowerBound(x_gray_code);
    x_gray_code_cnstr.constraint()->UpdateUpperBound(x_gray_code);
    for (int j = 0; j < num_interval_y_; ++j) {
      Eigen::VectorXd y_gray_code = gray_codes_y.cast<double>().row(j).transpose();
      y_gray_code_cnstr.constraint()->UpdateLowerBound(y_gray_code);
      y_gray_code_cnstr.constraint()->UpdateUpperBound(y_gray_code);
      Eigen::Matrix<double, 3, 4> vertices;
      vertices << phi_x_(i), phi_x_(i), phi_x_(i+1), phi_x_(i+1),
                  phi_y_(j), phi_y_(j+1), phi_y_(j+1), phi_y_(j),
                  phi_x_(i) * phi_y_(j), phi_x_(i) * phi_y_(j+1), phi_x_(i+1) * phi_y_(j+1), phi_x_(i+1) * phi_y_(j);
      for (int k = 0; k < a.cols(); ++k) {
        cost.constraint()->UpdateCoefficients(a.col(k));
        GurobiSolver gurobi_solver;
        if (gurobi_solver.available()) {
          auto result = gurobi_solver.Solve(prog_);
          EXPECT_EQ(result, SolutionResult::kSolutionFound);
          Eigen::Matrix<double, 1, 4> cost_at_vertices = a.col(k).transpose() * vertices;
          EXPECT_NEAR(prog_.GetOptimalCost(), cost_at_vertices.minCoeff(), 1E-4);
        }
      }
    }
  }
}

INSTANTIATE_TEST_CASE_P(
    TestMixedIntegerUtil, BilinearProductMcCormickEnvelopeSOS2Test,
    ::testing::Combine(::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<int>{2, 3})));
}  // namespace
}  // namespace solvers
}  // namespace drake
