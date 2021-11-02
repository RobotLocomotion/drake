#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

GTEST_TEST(TestLQR, TestException) {
  // A double integrator
  Eigen::Matrix2d A;
  A << 0, 1, 0, 0;
  const Eigen::Vector2d B(0, 1);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 1, 1> R = Eigen::MatrixXd::Identity(1, 1);
  Eigen::Vector2d N = Eigen::Vector2d::Zero();

  DRAKE_EXPECT_NO_THROW(LinearQuadraticRegulator(A, B, Q, R, N));
  DRAKE_EXPECT_NO_THROW(LinearQuadraticRegulator(A, B, Q, R));

  // R is not positive definite, should throw exception.
  EXPECT_THROW(LinearQuadraticRegulator(
        A, B, Q, Eigen::Matrix<double, 1, 1>::Zero()), std::runtime_error);
  EXPECT_THROW(LinearQuadraticRegulator(
        A, B, Q, Eigen::Matrix<double, 1, 1>::Zero(), N), std::runtime_error);
}

void TestLQRAgainstKnownSolution(
    double tolerance,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& S_known,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  LinearQuadraticRegulatorResult result =
      LinearQuadraticRegulator(A, B, Q, R, N);
  EXPECT_TRUE(CompareMatrices(K_known, result.K, tolerance,
        MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(S_known, result.S, tolerance,
        MatrixCompareType::absolute));
}

void TestLQRLinearSystemAgainstKnownSolution(
    double tolerance,
    const LinearSystem<double>& sys,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  std::unique_ptr<LinearSystem<double>> linear_lqr =
      LinearQuadraticRegulator(sys, Q, R, N);

  int n = sys.A().rows();
  int m = sys.B().cols();
  EXPECT_TRUE(CompareMatrices(linear_lqr->A(),
                              Eigen::Matrix<double, 0, 0>::Zero(), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->B(),
                              Eigen::MatrixXd::Zero(0, n), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->C(),
                              Eigen::MatrixXd::Zero(m, 0), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->D(), -K_known, tolerance,
                              MatrixCompareType::absolute));
  EXPECT_EQ(linear_lqr->time_period(), sys.time_period());
}

void TestLQRAffineSystemAgainstKnownSolution(
    double tolerance,
    const LinearSystem<double>& sys,
    const Eigen::Ref<const Eigen::MatrixXd>& K_known,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero()) {
  int n = sys.A().rows();
  int m = sys.B().cols();

  auto context = sys.CreateDefaultContext();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(m);

  sys.get_input_port().FixValue(context.get(), u0);
  if (sys.time_period() == 0.0) {
    context->SetContinuousState(x0);
  } else {
    context->SetDiscreteState(0, x0);
  }
  std::unique_ptr<AffineSystem<double>> lqr =
      LinearQuadraticRegulator(sys, *context, Q, R, N);

  EXPECT_TRUE(CompareMatrices(lqr->A(), Eigen::Matrix<double, 0, 0>::Zero(),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->B(), Eigen::MatrixXd::Zero(0, n),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->f0(), Eigen::Matrix<double, 0, 1>::Zero(),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->C(), Eigen::MatrixXd::Zero(m, 0),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->D(), -K_known,
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->y0(), u0 + K_known * x0,
                              tolerance, MatrixCompareType::absolute));
  EXPECT_EQ(lqr->time_period(), sys.time_period());
}

GTEST_TEST(TestLQR, DoubleIntegrator) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());

  // Trivial cost:
  Eigen::Matrix2d Q;
  Eigen::Matrix<double, 1, 1> R;
  Q << 1, 0, 0, 1;
  R << 1;

  // Analytical solution
  Eigen::Matrix<double, 1, 2> K;
  K << 1, std::sqrt(3);

  Eigen::Matrix<double, 2, 2> S;
  S << std::sqrt(3), 1, 1, std::sqrt(3);

  double tol = 1e-10;

  TestLQRAgainstKnownSolution(tol, K, S, A, B, Q, R);

  // Test LinearSystem version of the LQR
  TestLQRLinearSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  TestLQRAffineSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // A different cost function with the same Q and R, and an extra N = [1; 0].
  Eigen::Vector2d N(1, 0);
  // Known solution
  K = Eigen::Vector2d(1, 1);
  S = Eigen::Matrix2d::Identity();
  TestLQRAgainstKnownSolution(tol, K, S, A, B, Q, R, N);

  // Test LinearSystem version of the LQR
  TestLQRLinearSystemAgainstKnownSolution(tol, sys, K, Q, R, N);

  // Test AffineSystem version of the LQR
  TestLQRAffineSystemAgainstKnownSolution(tol, sys, K, Q, R, N);
}

GTEST_TEST(TestLQR, DiscreteDoubleIntegrator) {
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, 1, 0, 1;
  B << 0, 1;

  // Trivial cost:
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d::Identity();

  // Solution from dlqr in Matlab.
  Eigen::RowVector2d K;
  K << 0.422082440385453, 1.243928853903714;

  Eigen::Matrix2d S;
  // clang-format off
  S << 2.947122966707012, 2.369205407092467,
       2.369205407092467, 4.613134260996183;
  // clang-format on

  LinearQuadraticRegulatorResult result =
      DiscreteTimeLinearQuadraticRegulator(A, B, Q, R);

  const double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(result.K, K, tol));
  EXPECT_TRUE(CompareMatrices(result.S, S, tol));

  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero(), 0.1);

  // Test LinearSystem version of the LQR
  TestLQRLinearSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // Test AffineSystem version of the LQR
  TestLQRAffineSystemAgainstKnownSolution(tol, sys, K, Q, R);
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
