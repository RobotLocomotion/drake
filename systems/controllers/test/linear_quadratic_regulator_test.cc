#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

GTEST_TEST(TestLqr, TestException) {
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
  EXPECT_THROW(
      LinearQuadraticRegulator(A, B, Q, Eigen::Matrix<double, 1, 1>::Zero()),
      std::runtime_error);
  EXPECT_THROW(
      LinearQuadraticRegulator(A, B, Q, Eigen::Matrix<double, 1, 1>::Zero(), N),
      std::runtime_error);
}

void TestLqrAgainstKnownSolution(
    double tolerance, const Eigen::Ref<const Eigen::MatrixXd>& K_known,
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

void TestLqrLinearSystemAgainstKnownSolution(
    double tolerance, const LinearSystem<double>& sys,
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
  EXPECT_TRUE(CompareMatrices(linear_lqr->B(), Eigen::MatrixXd::Zero(0, n),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->C(), Eigen::MatrixXd::Zero(m, 0),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(linear_lqr->D(), -K_known, tolerance,
                              MatrixCompareType::absolute));
  EXPECT_EQ(linear_lqr->time_period(), sys.time_period());
}

void TestLqrAffineSystemAgainstKnownSolution(
    double tolerance, const LinearSystem<double>& sys,
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
  EXPECT_TRUE(CompareMatrices(lqr->B(), Eigen::MatrixXd::Zero(0, n), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->f0(), Eigen::Matrix<double, 0, 1>::Zero(),
                              tolerance, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->C(), Eigen::MatrixXd::Zero(m, 0), tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->D(), -K_known, tolerance,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(lqr->y0(), u0 + K_known * x0, tolerance,
                              MatrixCompareType::absolute));
  EXPECT_EQ(lqr->time_period(), sys.time_period());
}

// Test if the LQR solution satisfies the HJB equality
// minᵤ xᵀQx + uᵀRu + 2xᵀNu + 2xᵀS(Ax+Bu) = 0
void TestLqrWithHjb(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N = Eigen::MatrixXd(0, 0),
    const Eigen::Ref<const Eigen::MatrixXd>& F = Eigen::MatrixXd(0, 0)) {
  const auto lqr_result = LinearQuadraticRegulator(A, B, Q, R, N, F);
  const int nx = A.rows();
  const int nu = B.cols();
  // We first try to remove the constraint F*x = 0 by considering a new slack
  // variable y, where y is in the null-space of F, namely y = Pᵀx. We then
  // define the dynamics and cost using this new variable y.
  // minᵤ yᵀQy*y + uᵀRy*u + 2xᵀNy*u + 2xᵀSy(Ay*x+By*u) = 0
  Eigen::MatrixXd Ay = A;
  Eigen::MatrixXd By = B;
  Eigen::MatrixXd Qy = Q;
  Eigen::MatrixXd Ry = R;
  Eigen::MatrixXd Ny = N.rows() == 0 ? Eigen::MatrixXd::Zero(nx, nu) : N;
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(nx, nx);
  if (F.rows() != 0) {
    // First find P
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_F(F.transpose());
    ASSERT_EQ(qr_F.info(), Eigen::Success);
    const Eigen::MatrixXd F_Q = qr_F.matrixQ();
    P = F_Q.rightCols(nx - qr_F.rank()).transpose();
    Ay = P * A * P.transpose();
    By = P * B;
    Qy = P * Q * P.transpose();
    Ry = R;
    Ny = P * Ny;
  }
  const Eigen::MatrixXd Pt = P.transpose();
  const int ny = Pt.cols();
  // The minimization over u on the quadratic function occurs where the gradient
  // w.r.t u is 0. Ru = −(ByᵀSy + Nyᵀ)y
  // Since u = -Kx, we have Ru = -R*K*x = -R*K*Pᵀy
  // Hence R*K*Pᵀ= ByᵀSy + Nyᵀ. where Sy = P*Sx*Pᵀ
  const Eigen::MatrixXd Sy = P * lqr_result.S * Pt;
  const double tol = 1E-9;
  EXPECT_TRUE(CompareMatrices(Ry * lqr_result.K * Pt,
                              By.transpose() * Sy + Ny.transpose(), tol));
  // Now make sure that by plugging in u = -Kx = -KPᵀy, the left hand side of
  // HJB is zero.
  const Eigen::MatrixXd Ky = lqr_result.K * Pt;
  EXPECT_TRUE(CompareMatrices(Qy + Ky.transpose() * Ry * Ky - Ny * Ky -
                                  Ky.transpose() * Ny.transpose() + Sy * Ay +
                                  Ay.transpose() * Sy - Sy * By * Ky -
                                  Ky.transpose() * By.transpose() * Sy,
                              Eigen::MatrixXd::Zero(ny, ny), tol));
}

GTEST_TEST(TestLqr, DoubleIntegrator) {
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

  TestLqrAgainstKnownSolution(tol, K, S, A, B, Q, R);

  // Test LinearSystem version of the LQR
  TestLqrLinearSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // Call it as a generic System (by passing in a Context).
  // Should get the same result, but as an affine system.
  TestLqrAffineSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // A different cost function with the same Q and R, and an extra N = [1; 0].
  Eigen::Vector2d N(1, 0);
  // Known solution
  K = Eigen::Vector2d(1, 1);
  S = Eigen::Matrix2d::Identity();
  TestLqrAgainstKnownSolution(tol, K, S, A, B, Q, R, N);

  // Test LinearSystem version of the LQR
  TestLqrLinearSystemAgainstKnownSolution(tol, sys, K, Q, R, N);

  // Test AffineSystem version of the LQR
  TestLqrAffineSystemAgainstKnownSolution(tol, sys, K, Q, R, N);

  TestLqrWithHjb(A, B, Q, R, N);
}

GTEST_TEST(TestLqr, ConstrainedLinearSystem) {
  // Test the LQR for a constrained system
  // ẋ = Ax+Bu
  // Fx = 0
  Eigen::Matrix3d A;
  // clang-format off
  A << 1, 0, 2,
       0, 1, 3,
       0, 2, -1;
  // clang-format on
  Eigen::Matrix<double, 3, 2> B;
  // clang-format off
  B << 1, 3,
       0, 2,
       1, -1;
  // clang-format on
  const Eigen::RowVector3d F(1, 2, -3);
  Eigen::Matrix3d Q;
  // clang-format off
  Q << 4, 0, 2,
       0, 9, 3,
       2, 3, 8;
  // clang-format on
  Eigen::Matrix2d R;
  // clang-format off
  R << 1, 3,
       3, 10;
  // clang-format on
  Eigen::Matrix<double, 3, 2> N;
  N << 1, 0.5, 0.5, -1, 0, 1;

  TestLqrWithHjb(A, B, Q, R);
  TestLqrWithHjb(A, B, Q, R, N);
  TestLqrWithHjb(A, B, Q, R, Eigen::MatrixXd(0, 0), F);
  TestLqrWithHjb(A, B, Q, R, N, F);
}

GTEST_TEST(TestLqr, DiscreteDoubleIntegrator) {
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
  TestLqrLinearSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // Test AffineSystem version of the LQR
  TestLqrAffineSystemAgainstKnownSolution(tol, sys, K, Q, R);

  // A different cost function with the same Q and R, and an extra N = [1; 0].
  Eigen::Vector2d N(1, 0);
  // Solution from dlqr in Matlab.
  K << 0.427961322156271, 1.06165953563278;

  // clang-format off
  S << 2.48073711494216, 1.33665975925470,
       1.33665975925470, 4.45997883052027;
  // clang-format on

  result = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R, N);
  EXPECT_TRUE(CompareMatrices(result.K, K, tol));
  EXPECT_TRUE(CompareMatrices(result.S, S, tol));

  // Test LinearSystem version of the LQR
  TestLqrLinearSystemAgainstKnownSolution(tol, sys, K, Q, R, N);

  // Test AffineSystem version of the LQR
  TestLqrAffineSystemAgainstKnownSolution(tol, sys, K, Q, R, N);
}

// Adds test coverage for calling LQR from a LeafSystem and from a
// MultibodyPlant.
GTEST_TEST(TestLqr, AcrobotTest) {
  const examples::acrobot::AcrobotPlant<double> plant;
  auto context = plant.CreateDefaultContext();

  // Set nominal state to the upright fixed point.
  examples::acrobot::AcrobotState<double>& state =
      plant.get_mutable_state(context.get());
  state.set_theta1(M_PI);
  state.set_theta2(0.0);
  state.set_theta1dot(0.0);
  state.set_theta2dot(0.0);

  plant.GetInputPort("elbow_torque").FixValue(context.get(), 0.0);

  const Eigen::Matrix4d Q = Eigen::Vector4d(10.0, 10.0, 1.0, 1.0).asDiagonal();
  const Vector1d R(1);
  const auto controller = LinearQuadraticRegulator(plant, *context, Q, R);

  // Confirm that I get the same result by linearizing explicitly.
  const auto linear_system = Linearize(plant, *context);
  const auto lqr_result =
      LinearQuadraticRegulator(linear_system->A(), linear_system->B(), Q, R);
  EXPECT_TRUE(CompareMatrices(-lqr_result.K, controller->D(), 1e-12));

  // Confirm that I get the same result via MultibodyPlant.
  multibody::MultibodyPlant<double> mbp(0.0);
  multibody::Parser(&mbp).AddModels(
      FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf"));
  mbp.Finalize();
  auto mbp_context = mbp.CreateDefaultContext();
  mbp.SetPositions(mbp_context.get(), Eigen::Vector2d(M_PI, 0));
  mbp.get_actuation_input_port().FixValue(mbp_context.get(), 0.0);
  const auto mbp_controller = LinearQuadraticRegulator(
      mbp, *mbp_context, Q, R, Eigen::Matrix<double, 4, 1>::Zero(),
      mbp.get_actuation_input_port().get_index());

  EXPECT_TRUE(CompareMatrices(mbp_controller->D(), controller->D(), 1e-9));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
