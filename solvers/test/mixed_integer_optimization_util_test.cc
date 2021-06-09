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
  static_assert(std::is_same_v<decltype(y1), VectorXDecisionVariable>,
                "y1 should be a dynamic-sized vector.");
  auto lambda2 = prog.NewContinuousVariables<3>("lambda2");
  auto y2 =
      AddLogarithmicSos2Constraint(&prog, lambda2.cast<symbolic::Expression>());
  static_assert(std::is_same_v<decltype(y2), VectorDecisionVariable<1>>,
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
    y_assignment.evaluator()->UpdateLowerBound(y_val);
    y_assignment.evaluator()->UpdateUpperBound(y_val);

    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog, {}, {});
      EXPECT_TRUE(result.is_success());
      const auto lambda_val = result.GetSolution(lambda);
      Eigen::VectorXd lambda_val_expected = Eigen::VectorXd::Zero(num_lambda);
      lambda_val_expected(i) = 0.5;
      lambda_val_expected(i + 1) = 0.5;
      EXPECT_TRUE(CompareMatrices(lambda_val, lambda_val_expected, 1E-5,
                                  MatrixCompareType::absolute));
    }
  }
}

GTEST_TEST(TestSos2, TestClosestPointOnLineSegments) {
  // We will define line segments A₀A₁, ..., A₅A₆ in 2D, where points Aᵢ are
  // defined as A₀ = (0, 0), A₁ = (1, 1), A₂ = (2, 0), A₃ = (4, 2), A₅ = (6, 0),
  // A₅ = (7, 1), A₆ = (8, 0). We compute the closest point P = (x, y) on the
  // line segments to a given point Q.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>()(0);
  auto y = prog.NewContinuousVariables<1>()(0);
  Eigen::Matrix<double, 2, 7> A;
  // clang-format off
  A << 0, 1, 2, 4, 6, 7, 8,
       0, 1, 0, 2, 0, 1, 0;
  // clang-format on
  auto lambda = prog.NewContinuousVariables<7>();
  auto z = prog.NewBinaryVariables<6>();
  AddSos2Constraint(&prog, lambda.cast<symbolic::Expression>(),
                    z.cast<symbolic::Expression>());
  const Vector2<symbolic::Expression> line_segment = A * lambda;
  prog.AddLinearConstraint(line_segment(0) == x);
  prog.AddLinearConstraint(line_segment(1) == y);

  // Add a dummy cost function, which we will change in the for loop below.
  Binding<QuadraticCost> cost =
      prog.AddQuadraticCost(Eigen::Matrix2d::Zero(), Eigen::Vector2d::Zero(), 0,
                            VectorDecisionVariable<2>(x, y));
  // We will test with different points Qs, each Q corresponds to a nearest
  // point P on the line segments.
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> Q_and_P;
  Q_and_P.push_back(
      std::make_pair(Eigen::Vector2d(1, 1), Eigen::Vector2d(1, 1)));
  Q_and_P.push_back(
      std::make_pair(Eigen::Vector2d(1.9, 1), Eigen::Vector2d(1.45, 0.55)));
  Q_and_P.push_back(
      std::make_pair(Eigen::Vector2d(3, 1), Eigen::Vector2d(3, 1)));
  Q_and_P.push_back(
      std::make_pair(Eigen::Vector2d(5, 1.2), Eigen::Vector2d(4.9, 1.1)));
  Q_and_P.push_back(
      std::make_pair(Eigen::Vector2d(7.5, 1.2), Eigen::Vector2d(7.15, 0.85)));
  for (const auto& QP_pair : Q_and_P) {
    const Eigen::Vector2d& Q = QP_pair.first;
    // The cost is |P-Q|²
    cost.evaluator()->UpdateCoefficients(2 * Eigen::Matrix2d::Identity(),
                                         -2 * Q, Q.squaredNorm());

    // Any mixed integer convex solver can solve this problem, here we choose
    // gurobi.
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog, {}, {});
      EXPECT_TRUE(result.is_success());
      const Eigen::Vector2d P(
          result.GetSolution(VectorDecisionVariable<2>(x, y)));
      const Eigen::Vector2d P_expected = QP_pair.second;
      EXPECT_TRUE(CompareMatrices(P, P_expected, 1E-6));
      EXPECT_NEAR(result.get_optimal_cost(), (Q - P_expected).squaredNorm(),
                  1E-12);
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
    binary_assignment.evaluator()->UpdateLowerBound(code);
    binary_assignment.evaluator()->UpdateUpperBound(code);
    GurobiSolver gurobi_solver;
    if (gurobi_solver.available()) {
      auto result = gurobi_solver.Solve(prog, {}, {});
      EXPECT_TRUE(result.is_success());
      Eigen::VectorXd lambda_expected(num_lambda);
      lambda_expected.setZero();
      lambda_expected(i) = 1;
      EXPECT_TRUE(CompareMatrices(result.GetSolution(lambda), lambda_expected,
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

GTEST_TEST(TestLogarithmicSos1, Test) {
  MathematicalProgram prog;
  VectorX<symbolic::Variable> y, lambda;
  std::tie(lambda, y) = AddLogarithmicSos1Constraint(&prog, 3);
  EXPECT_EQ(lambda.rows(), 3);
  EXPECT_EQ(y.rows(), 2);

  auto check = [&prog, &y, &lambda](const Eigen::VectorXd& lambda_val,
                                    const Eigen::VectorXd& y_val,
                                    bool satisfied_expected) {
    Eigen::VectorXd x_val = Eigen::VectorXd::Zero(prog.num_vars());
    prog.SetDecisionVariableValueInVector(y, y_val, &x_val);
    prog.SetDecisionVariableValueInVector(lambda, lambda_val, &x_val);
    bool satisfied = true;
    for (const auto& binding : prog.GetAllConstraints()) {
      satisfied =
          satisfied && binding.evaluator()->CheckSatisfied(
                           prog.GetBindingVariableValues(binding, x_val));
    }
    EXPECT_EQ(satisfied, satisfied_expected);
  };
  check(Eigen::Vector3d(0, 0, 1), Eigen::Vector2d(1, 1), true);
  check(Eigen::Vector3d(1, 0, 0), Eigen::Vector2d(1, 0), false);
  check(Eigen::Vector3d(0, 1, 0), Eigen::Vector2d(0, 1), true);
  check(Eigen::Vector3d(0, 0.5, 0.5), Eigen::Vector2d(1, 1), false);
  check(Eigen::Vector3d(0, 0.1, 1), Eigen::Vector2d(1, 0), false);
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
      std::is_same_v<decltype(lambda1), MatrixDecisionVariable<3, 4>>,
      "lambda should be a static matrix");

  auto lambda2 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_dynamic, phi_y_static, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(std::is_same_v<decltype(lambda2),
                               MatrixDecisionVariable<Eigen::Dynamic, 4>>,
                "lambda's type is incorrect");

  auto lambda3 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_static, phi_y_dynamic, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(std::is_same_v<decltype(lambda3),
                               MatrixDecisionVariable<3, Eigen::Dynamic>>,
                "lambda's type is incorrect");

  auto lambda4 = AddBilinearProductMcCormickEnvelopeSos2(
      &prog, x, y, w, phi_x_dynamic, phi_y_dynamic, Bx, By,
      IntervalBinning::kLogarithmic);
  static_assert(
      std::is_same_v<
          decltype(lambda4),
          MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic>>,
      "lambda's type is incorrect");
}

class BilinearProductMcCormickEnvelopeTest {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BilinearProductMcCormickEnvelopeTest)

  BilinearProductMcCormickEnvelopeTest(int num_interval_x, int num_interval_y)
      : prog_{},
        prog_result_{},
        num_interval_x_{num_interval_x},
        num_interval_y_{num_interval_y},
        w_{prog_.NewContinuousVariables<1>()(0)},
        x_{prog_.NewContinuousVariables<1>()(0)},
        y_{prog_.NewContinuousVariables<1>()(0)},
        phi_x_{Eigen::VectorXd::LinSpaced(num_interval_x_ + 1, 0, 1)},
        phi_y_{Eigen::VectorXd::LinSpaced(num_interval_y_ + 1, 0, 1)} {}

  virtual ~BilinearProductMcCormickEnvelopeTest() = default;

  virtual Eigen::VectorXd SetBinaryValue(int active_interval,
                                         int num_interval) const = 0;

  Eigen::VectorXd SetBinaryValueLinearBinning(int active_interval,
                                              int num_interval) const {
    Eigen::VectorXd b = Eigen::VectorXd::Zero(num_interval);
    b(active_interval) = 1;
    return b;
  }

  void TestFeasiblePoint() {
    auto x_constraint = prog_.AddBoundingBoxConstraint(0, 0, x_);
    auto y_constraint = prog_.AddBoundingBoxConstraint(0, 0, y_);
    auto w_constraint = prog_.AddBoundingBoxConstraint(0, 0, w_);

    auto CheckFeasibility = [&x_constraint, &y_constraint, &w_constraint](
        MathematicalProgram* prog, double x_val, double y_val, double w_val,
        bool is_feasible) {
      auto UpdateBound = [](Binding<BoundingBoxConstraint>* constraint,
                            double val) {
        constraint->evaluator()->UpdateLowerBound(Vector1d(val));
        constraint->evaluator()->UpdateUpperBound(Vector1d(val));
      };
      UpdateBound(&x_constraint, x_val);
      UpdateBound(&y_constraint, y_val);
      UpdateBound(&w_constraint, w_val);

      prog->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);

      GurobiSolver solver;
      if (solver.available()) {
        MathematicalProgramResult result;
        solver.Solve(*prog, {}, {}, &result);
        EXPECT_EQ(result.get_solution_result(),
                  is_feasible ? SolutionResult::kSolutionFound
                              : SolutionResult::kInfeasibleConstraints);
      }
    };

    // Feasible points
    CheckFeasibility(&prog_, 0, 0, 0, true);
    CheckFeasibility(&prog_, 0, 1, 0, true);
    CheckFeasibility(&prog_, 1, 0, 0, true);
    CheckFeasibility(&prog_, 1, 1, 1, true);
    CheckFeasibility(&prog_, 0.5, 1, 0.5, true);
    CheckFeasibility(&prog_, 1, 0.5, 0.5, true);
    CheckFeasibility(&prog_, 0.5, 0.5, 0.25, true);

    if (num_interval_x_ == 2 && num_interval_y_ == 2) {
      CheckFeasibility(&prog_, 0.5, 0.5, 0.26, false);
      CheckFeasibility(&prog_, 0.25, 0.25, 0.1, true);
      CheckFeasibility(&prog_, 0.25, 0.25, 0.126, false);
      CheckFeasibility(&prog_, 0.5, 0.6, 0.301, false);
    }
    if (num_interval_x_ == 3 && num_interval_y_ == 3) {
      CheckFeasibility(&prog_, 1.0 / 3, 1.0 / 3, 0.1, false);
      CheckFeasibility(&prog_, 0.5, 0.5, 0.26, true);
      CheckFeasibility(&prog_, 1.0 / 3, 2.0 / 3, 2.0 / 9, true);
      CheckFeasibility(&prog_, 0.5, 0.5, 5.0 / 18, true);
      CheckFeasibility(&prog_, 0.5, 0.5, 5.0 / 18 + 0.001, false);
    }
  }

  void TestLinearObjective() {
    // We will assign the binary variables Bx_ and By_ to determine which
    // interval is active. If we use logarithmic binning, then Bx_ and By_ take
    // values in the gray code, representing integer i and j, such that x is
    // constrained in [φx(i), φx(i+1)], y is constrained in [φy(j), φy(j+1)].
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
      const auto Bx_val = SetBinaryValue(i, num_interval_x_);

      Bx_constraint.evaluator()->UpdateLowerBound(Bx_val);
      Bx_constraint.evaluator()->UpdateUpperBound(Bx_val);
      for (int j = 0; j < num_interval_y_; ++j) {
        const auto By_val = SetBinaryValue(j, num_interval_y_);
        By_constraint.evaluator()->UpdateLowerBound(By_val);
        By_constraint.evaluator()->UpdateUpperBound(By_val);

        // vertices.col(l) is the l'th vertex of the tetrahedron.
        Eigen::Matrix<double, 3, 4> vertices;
        vertices.row(0) << phi_x_(i), phi_x_(i), phi_x_(i + 1), phi_x_(i + 1);
        vertices.row(1) << phi_y_(j), phi_y_(j + 1), phi_y_(j), phi_y_(j + 1);
        vertices.row(2) = vertices.row(0).cwiseProduct(vertices.row(1));
        for (int k = 0; k < a.cols(); ++k) {
          cost.evaluator()->UpdateCoefficients(a.col(k));
          GurobiSolver gurobi_solver;
          if (gurobi_solver.available()) {
            gurobi_solver.Solve(prog_, {}, {}, &prog_result_);
            EXPECT_TRUE(prog_result_.is_success());
            Eigen::Matrix<double, 1, 4> cost_at_vertices =
                a.col(k).transpose() * vertices;
            EXPECT_NEAR(prog_result_.get_optimal_cost(),
                        cost_at_vertices.minCoeff(), 1E-4);

            TestLinearObjectiveCheck(i, j, k);
          }
        }
      }
    }
  }

  virtual void TestLinearObjectiveCheck(int i, int j, int k) const {}

 protected:
  MathematicalProgram prog_;
  MathematicalProgramResult prog_result_;
  const int num_interval_x_;
  const int num_interval_y_;
  const symbolic::Variable w_;
  const symbolic::Variable x_;
  const symbolic::Variable y_;
  const Eigen::VectorXd phi_x_;
  const Eigen::VectorXd phi_y_;
  VectorXDecisionVariable Bx_;
  VectorXDecisionVariable By_;
};

class BilinearProductMcCormickEnvelopeSos2Test
    : public ::testing::TestWithParam<std::tuple<int, int, IntervalBinning>>,
      public BilinearProductMcCormickEnvelopeTest {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BilinearProductMcCormickEnvelopeSos2Test)

  BilinearProductMcCormickEnvelopeSos2Test()
      : BilinearProductMcCormickEnvelopeTest(std::get<0>(GetParam()),
                                             std::get<1>(GetParam())),
        binning_{std::get<2>(GetParam())},
        Bx_size_{binning_ == IntervalBinning::kLogarithmic
                     ? CeilLog2(num_interval_x_)
                     : num_interval_x_},
        By_size_{binning_ == IntervalBinning::kLogarithmic
                     ? CeilLog2(num_interval_y_)
                     : num_interval_y_} {
    Bx_ = prog_.NewBinaryVariables(Bx_size_);
    By_ = prog_.NewBinaryVariables(By_size_);
    lambda_ = AddBilinearProductMcCormickEnvelopeSos2(
        &prog_, x_, y_, w_, phi_x_, phi_y_, Bx_.cast<symbolic::Expression>(),
        By_.cast<symbolic::Expression>(), binning_);
  }

  Eigen::VectorXd SetBinaryValueLogarithmicBinning(int active_interval,
                                                   int num_interval) const {
    const Eigen::MatrixXi gray_codes =
        math::CalculateReflectedGrayCodes(solvers::CeilLog2(num_interval));
    return gray_codes.row(active_interval).cast<double>().transpose();
  }

  Eigen::VectorXd SetBinaryValue(int active_interval,
                                 int num_interval) const override {
    switch (binning_) {
      case IntervalBinning::kLinear: {
        return SetBinaryValueLinearBinning(active_interval, num_interval);
      }
      case IntervalBinning::kLogarithmic: {
        return SetBinaryValueLogarithmicBinning(active_interval, num_interval);
      }
      default: {
        throw std::runtime_error(
            "This default case should not be reached. We add the default case "
            "due to a gcc-5 pitfall.");
      }
    }
  }

  void TestLinearObjectiveCheck(int i, int j, int k) const override {
    // Check that λ has the correct value, except λ(i, j), λ(i, j+1),
    // λ(i+1, j) and λ(i+1, j+1), all other entries in λ are zero.
    double w{0};
    double x{0};
    double y{0};
    for (int m = 0; m <= num_interval_x_; ++m) {
      for (int n = 0; n <= num_interval_y_; ++n) {
        if (!((m == i && n == j) || (m == i && n == (j + 1)) ||
              (m == (i + 1) && n == j) || (m == (i + 1) && n == (j + 1)))) {
          EXPECT_NEAR(prog_result_.GetSolution(lambda_(m, n)), 0, 1E-5);
        } else {
          double lambda_mn{prog_result_.GetSolution(lambda_(m, n))};
          x += lambda_mn * phi_x_(m);
          y += lambda_mn * phi_y_(n);
          w += lambda_mn * phi_x_(m) * phi_y_(n);
        }
      }
    }
    EXPECT_NEAR(prog_result_.GetSolution(x_), x, 1E-4);
    EXPECT_NEAR(prog_result_.GetSolution(y_), y, 1E-4);
    EXPECT_NEAR(prog_result_.GetSolution(w_), w, 1E-4);
  }

 protected:
  const IntervalBinning binning_;
  const int Bx_size_;
  const int By_size_;
  MatrixXDecisionVariable lambda_;
};

TEST_P(BilinearProductMcCormickEnvelopeSos2Test, LinearObjectiveTest) {
  TestLinearObjective();
}

TEST_P(BilinearProductMcCormickEnvelopeSos2Test, FeasiblePointTest) {
  TestFeasiblePoint();
}

INSTANTIATE_TEST_SUITE_P(
    TestMixedIntegerUtil, BilinearProductMcCormickEnvelopeSos2Test,
    ::testing::Combine(::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<IntervalBinning>{
                           IntervalBinning::kLogarithmic,
                           IntervalBinning::kLinear})));

class BilinearProductMcCormickEnvelopeMultipleChoiceTest
    : public ::testing::TestWithParam<std::tuple<int, int>>,
      public BilinearProductMcCormickEnvelopeTest {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(
      BilinearProductMcCormickEnvelopeMultipleChoiceTest)

  BilinearProductMcCormickEnvelopeMultipleChoiceTest()
      : BilinearProductMcCormickEnvelopeTest(std::get<0>(GetParam()),
                                             std::get<1>(GetParam())) {
    Bx_ = prog_.NewBinaryVariables(num_interval_x_);
    By_ = prog_.NewBinaryVariables(num_interval_y_);
    AddBilinearProductMcCormickEnvelopeMultipleChoice(
        &prog_, x_, y_, w_, phi_x_, phi_y_, Bx_.cast<symbolic::Expression>(),
        By_.cast<symbolic::Expression>());
  }

  Eigen::VectorXd SetBinaryValue(int active_interval,
                                 int num_interval) const override {
    return SetBinaryValueLinearBinning(active_interval, num_interval);
  }
};

TEST_P(BilinearProductMcCormickEnvelopeMultipleChoiceTest,
       LinearObjectiveTest) {
  TestLinearObjective();
}

TEST_P(BilinearProductMcCormickEnvelopeMultipleChoiceTest, FeasiblePointTest) {
  TestFeasiblePoint();
}

INSTANTIATE_TEST_SUITE_P(
    TestMixedIntegerUtil, BilinearProductMcCormickEnvelopeMultipleChoiceTest,
    ::testing::Combine(::testing::ValuesIn(std::vector<int>{2, 3}),
                       ::testing::ValuesIn(std::vector<int>{2, 3})));
}  // namespace
}  // namespace solvers
}  // namespace drake
