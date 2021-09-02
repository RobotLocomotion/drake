#include "drake/multibody/optimization/quaternion_integration_constraint.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
const double kEps = std::numeric_limits<double>::epsilon();

GTEST_TEST(TestDiffferentiableNorm, test_double) {
  EXPECT_NEAR(internal::DifferentiableNorm(Eigen::Vector3d(1, 0, 0)), 1., kEps);
}

GTEST_TEST(TestDiffferentiableNorm, test_autodiff) {
  Eigen::Matrix3Xd x_grad(3, 1);
  x_grad << 1, 2, 3;
  Vector3<AutoDiffXd> x = math::InitializeAutoDiff(
      Eigen::Vector3d(1, 0, 0), x_grad);
  AutoDiffXd norm = internal::DifferentiableNorm(x);
  AutoDiffXd norm_expected = x.norm();
  EXPECT_NEAR(norm.value(), norm_expected.value(), 10 * kEps);
  EXPECT_TRUE(CompareMatrices(norm.derivatives(), norm_expected.derivatives(),
                              10 * kEps));

  // Test when x is zero.
  x = math::InitializeAutoDiff(Eigen::Vector3d::Zero(), x_grad);
  norm = internal::DifferentiableNorm(x);
  EXPECT_NEAR(norm.value(), 0., 10 * kEps);
  EXPECT_TRUE(CompareMatrices(norm.derivatives(), Vector1d::Zero(), 10 * kEps));
}

// Evaluate the left-hand side of the constraint
//
//     If allow_quaternion_negation = true:
//     (z₂ • (Δz⊗z₁))² = 1
//     else
//     z₂ • (Δz⊗z₁) = 1
// where Δz = [cos(|ω|h/2), ω/|ω|sin(|ω|h/2)]
// We compute the term Δz⊗z₁ using an alternative approach
// Δz⊗z₁ = exp(Ah/2)*z₁
// where A is the skew-symmetric matrix
// A = [0, -ω_x, -ω_y, -ω_z]
//     [ω_x,  0, -ω_z,  ω_y]
//     [ω_y, ω_z,   0, -ω_x]
//     [ω_z, -ω_y, ω_x,   0]
// We can show that exp(Ah) = cos(|ω|h/2)* I + sin(|ω|h/2)/|ω| * A
template <typename T>
Vector1<T> EvalQuaternionIntegration(
    const Eigen::Quaternion<T>& quat1, const Eigen::Quaternion<T>& quat2,
    const Eigen::Ref<const Vector3<T>>& angular_vel, const T& h,
    bool allow_quaternion_negation) {
  Vector1<T> ret;
  using std::cos;
  using std::sin;
  const T angular_vel_norm = internal::DifferentiableNorm<T>(angular_vel);
  Matrix4<T> A;
  // clang-format off
  A << 0, -angular_vel(0), -angular_vel(1), -angular_vel(2),
       angular_vel(0), 0, -angular_vel(2), angular_vel(1),
       angular_vel(1), angular_vel(2), 0, -angular_vel(0),
       angular_vel(2), -angular_vel(1), angular_vel(0), 0;
  // clang-format on
  Matrix4<T> exp_half_Ah;
  if (ExtractDoubleOrThrow(angular_vel_norm) == 0) {
    exp_half_Ah =
        cos(angular_vel_norm * h / 2) * Eigen::Matrix4d::Identity() + h / 2 * A;
  } else {
    exp_half_Ah = cos(angular_vel_norm * h / 2) * Eigen::Matrix4d::Identity() +
                  sin(angular_vel_norm * h / 2) / angular_vel_norm * A;
  }
  const Vector4<T> delta_z_times_quat1_vec4 =
      exp_half_Ah * Vector4<T>(quat1.w(), quat1.x(), quat1.y(), quat1.z());
  const Eigen::Quaternion<T> delta_z_times_quat1(
      delta_z_times_quat1_vec4(0), delta_z_times_quat1_vec4(1),
      delta_z_times_quat1_vec4(2), delta_z_times_quat1_vec4(3));
  if (allow_quaternion_negation) {
    using std::pow;
    ret(0) = pow(quat2.dot(delta_z_times_quat1), 2);
  } else {
    ret(0) = quat2.dot(delta_z_times_quat1);
  }
  return ret;
}

void TestEval(const QuaternionEulerIntegrationConstraint& dut,
              const Eigen::Vector4d& quat1, const Eigen::Vector4d& quat2,
              const Eigen::Vector3d& angular_vel, double h) {
  const auto x = dut.ComposeVariable<double>(quat1, quat2, angular_vel, h);
  Eigen::VectorXd y;
  dut.Eval(x, &y);
  const auto y_expected = EvalQuaternionIntegration<double>(
      Eigen::Quaterniond(quat1(0), quat1(1), quat1(2), quat1(3)),
      Eigen::Quaterniond(quat2(0), quat2(1), quat2(2), quat2(3)), angular_vel,
      h, dut.allow_quaternion_negation());
  EXPECT_TRUE(CompareMatrices(y, y_expected, 10 * kEps));

  // Check Eval with autodiff. Use arbitrary gradient.
  Eigen::MatrixXd x_grad(12, 2);
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 2; ++j) {
      x_grad(i, j) = 0.2 * i + 0.6 * j;
    }
  }
  const AutoDiffVecXd x_ad =
      math::InitializeAutoDiff(Eigen::VectorXd(x), x_grad);
  AutoDiffVecXd y_ad;
  dut.Eval(x_ad, &y_ad);
  const auto y_ad_expected = EvalQuaternionIntegration<AutoDiffXd>(
      Eigen::Quaternion<AutoDiffXd>(x_ad(0), x_ad(1), x_ad(2), x_ad(3)),
      Eigen::Quaternion<AutoDiffXd>(x_ad(4), x_ad(5), x_ad(6), x_ad(7)),
      x_ad.segment<3>(8), x_ad(11), dut.allow_quaternion_negation());
  EXPECT_TRUE(CompareMatrices(
      math::ExtractValue(y_ad),
      math::ExtractValue(y_ad_expected), 10 * kEps));
  EXPECT_TRUE(CompareMatrices(
      math::ExtractGradient(y_ad),
      math::ExtractGradient(y_ad_expected), 10 * kEps));
}

GTEST_TEST(QuaternionEulerIntegrationConstraintTest, TestEval) {
  for (bool allow_quaternion_negation : {true, false}) {
    const QuaternionEulerIntegrationConstraint dut{allow_quaternion_negation};
    EXPECT_EQ(dut.num_vars(), 12);
    EXPECT_EQ(dut.num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(dut.lower_bound(), Vector1d::Ones()));
    EXPECT_TRUE(CompareMatrices(dut.upper_bound(), Vector1d::Ones()));

    const Eigen::Vector4d quat1 =
        Eigen::Vector4d(0.3, 0.5, 0.2, 0.1).normalized();
    const Eigen::Vector4d quat2 =
        Eigen::Vector4d(1.3, -.5, -0.7, 0.6).normalized();
    const Eigen::Vector3d angular_vel(0.2, 0.5, -1.2);
    const double h{0.4};
    TestEval(dut, quat1, quat2, angular_vel, h);

    // Now test angular_vel = 0
    TestEval(dut, quat1, quat2, Eigen::Vector3d::Zero(), h);

    // Now test h = 0
    TestEval(dut, quat1, quat2, angular_vel, 0);
  }
}

GTEST_TEST(QuaternionEulerIntegrationConstraintTest, TestEvalSymbolic) {
  for (bool allow_quaternion_negation : {true, false}) {
    const QuaternionEulerIntegrationConstraint dut{allow_quaternion_negation};
    const symbolic::Variable z1_w("z1_w");
    const symbolic::Variable z1_x("z1_x");
    const symbolic::Variable z1_y("z1_y");
    const symbolic::Variable z1_z("z1_z");
    const symbolic::Variable z2_w("z2_w");
    const symbolic::Variable z2_x("z2_x");
    const symbolic::Variable z2_y("z2_y");
    const symbolic::Variable z2_z("z2_z");
    const symbolic::Variable omega_x("omega_x");
    const symbolic::Variable omega_y("omega_y");
    const symbolic::Variable omega_z("omega_z");
    const symbolic::Variable h("h");
    VectorX<symbolic::Expression> y;
    EXPECT_NO_THROW(
        dut.Eval(dut.ComposeVariable<symbolic::Variable>(
                     Vector4<symbolic::Variable>(z1_w, z1_x, z1_y, z1_z),
                     Vector4<symbolic::Variable>(z2_w, z2_x, z2_y, z2_z),
                     Vector3<symbolic::Variable>(omega_x, omega_y, omega_z), h),
                 &y));
  }
}

Eigen::Quaterniond ComputeNextQuaternion(const Eigen::Quaterniond& quat_curr,
                                         const Eigen::Vector3d& angular_vel,
                                         double h) {
  if (angular_vel.norm() == 0) {
    return quat_curr;
  } else {
    return Eigen::AngleAxisd(angular_vel.norm() * h, angular_vel.normalized()) *
           quat_curr;
  }
}

void ExpectTwoQuaternionsClose(const Eigen::Quaterniond& z1,
                               const Eigen::Quaterniond& z2, double tol,
                               bool allow_quaternion_negation) {
  // The two quaternions have to be unit-length.
  DRAKE_DEMAND(std::abs(z1.coeffs().norm() - 1) < 1E-5);
  DRAKE_DEMAND(std::abs(z2.coeffs().norm() - 1) < 1E-5);
  const double cos_half_angle = z1.dot(z2);
  if (allow_quaternion_negation) {
    EXPECT_NEAR(std::abs(cos_half_angle), 1, tol);
  } else {
    EXPECT_NEAR(cos_half_angle, 1, tol);
  }
}

void TestSolveNextQuaternion(bool allow_quaternion_negation) {
  // Fix quat1, angular_vel and h, solve for quat2 that satisfies the
  // constraint.
  solvers::MathematicalProgram prog{};
  auto quat1 = prog.NewContinuousVariables<4>();
  auto quat2 = prog.NewContinuousVariables<4>();
  auto angular_vel = prog.NewContinuousVariables<3>();
  auto h = prog.NewContinuousVariables<1>()(0);

  auto dut = std::make_shared<QuaternionEulerIntegrationConstraint>(
      allow_quaternion_negation);
  auto integration_cnstr = prog.AddConstraint(
      dut,
      dut->ComposeVariable<symbolic::Variable>(quat1, quat2, angular_vel, h));
  // Add the unit length constraint
  prog.AddConstraint(std::make_shared<UnitQuaternionConstraint>(), quat2);

  auto angular_vel_cnstr = prog.AddBoundingBoxConstraint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), angular_vel);
  auto quat1_cnstr = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(), quat1);
  auto h_cnstr = prog.AddBoundingBoxConstraint(0., 0., h);
  std::vector<Eigen::Vector4d> quat1_vals;
  quat1_vals.push_back(Eigen::Vector4d(1, 0, 0, 0).normalized());
  quat1_vals.push_back(Eigen::Vector4d(0.5, -0.2, 1., 0.3).normalized());
  quat1_vals.push_back(Eigen::Vector4d(1.5, -0.3, 0., -0.8).normalized());

  std::vector<Eigen::Vector3d> angular_vel_vals;
  angular_vel_vals.push_back(Eigen::Vector3d(0.5, 0.1, -0.2));
  angular_vel_vals.push_back(Eigen::Vector3d(-0.4, 1.5, 0.2));
  // angular_vel = 0 is a special case.
  angular_vel_vals.push_back(Eigen::Vector3d::Zero());
  // Also test very small angular velocity
  angular_vel_vals.push_back(Eigen::Vector3d::Ones() * kEps);
  angular_vel_vals.push_back(2 * Eigen::Vector3d::Ones() * kEps);

  // Our formulation should also work for negative time interval, hence try h =
  // -0.5
  std::vector<double> h_vals{{0.5, 0.4, 0.1, -0.5}};

  // The solvers might get stuck at local minima, hence we could try many
  // initial guesses. With our current formulation, the initial guess of (1, 0,
  // 0, 0) always succeeds. In the future if we change the formulation and the
  // initial guess of (1, 0, 0, 0) doesn't work, then append more initial
  // guesses here.
  std::vector<Eigen::Vector4d> quat2_initial_guesses;
  quat2_initial_guesses.emplace_back(1, 0, 0, 0);

  for (const auto& quat1_val : quat1_vals) {
    quat1_cnstr.evaluator()->set_bounds(quat1_val, quat1_val);
    for (const auto& angular_vel_val : angular_vel_vals) {
      angular_vel_cnstr.evaluator()->set_bounds(angular_vel_val,
                                                angular_vel_val);
      for (const auto& h_val : h_vals) {
        h_cnstr.evaluator()->set_bounds(Vector1d(h_val), Vector1d(h_val));
        solvers::MathematicalProgramResult result;
        for (const auto& quat2_init : quat2_initial_guesses) {
          prog.SetInitialGuess(quat2, quat2_init);
          result = solvers::Solve(prog);
          if (result.is_success()) {
            break;
          }
        }
        EXPECT_TRUE(result.is_success());
        // Now compute quat2 using quat1, angular_vel and h, compare that with
        // the solution from the optimization.
        const Eigen::Vector4d quat2_sol = result.GetSolution(quat2);
        const Eigen::Quaterniond quat2_expected = ComputeNextQuaternion(
            Eigen::Quaterniond(quat1_val(0), quat1_val(1), quat1_val(2),
                               quat1_val(3)),
            angular_vel_val, h_val);
        ExpectTwoQuaternionsClose(
            Eigen::Quaterniond(quat2_sol(0), quat2_sol(1), quat2_sol(2),
                               quat2_sol(3)),
            quat2_expected, 1E-5, allow_quaternion_negation);
      }
    }
  }
}

GTEST_TEST(QuaternionEulerIntegrationConstraintTest, SolveNextQuaternion) {
  TestSolveNextQuaternion(true);
  TestSolveNextQuaternion(false);
}

void TestSolveAngularVel(bool allow_quaternion_negation) {
  // Fix quat1, quat2 and h, solve for angular_vel that satisfies the
  // constraint.
  solvers::MathematicalProgram prog{};
  auto quat1 = prog.NewContinuousVariables<4>();
  auto quat2 = prog.NewContinuousVariables<4>();
  auto angular_vel = prog.NewContinuousVariables<3>();
  auto h = prog.NewContinuousVariables<1>()(0);

  auto dut = std::make_shared<QuaternionEulerIntegrationConstraint>(
      allow_quaternion_negation);
  prog.AddConstraint(dut, dut->ComposeVariable<symbolic::Variable>(
                              quat1, quat2, angular_vel, h));

  auto quat1_cnstr = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(), quat1);
  auto quat2_cnstr = prog.AddBoundingBoxConstraint(
      Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero(), quat2);
  auto h_cnstr = prog.AddBoundingBoxConstraint(0., 0., h);
  std::vector<Eigen::Vector4d> quat1_vals;
  quat1_vals.push_back(Eigen::Vector4d(1, 0, 0, 0).normalized());
  quat1_vals.push_back(Eigen::Vector4d(0.5, -0.2, 1., 0.3).normalized());
  quat1_vals.push_back(Eigen::Vector4d(1.5, -0.3, 0., -0.8).normalized());

  std::vector<Eigen::Vector4d> quat2_vals = quat1_vals;

  // Our formulation should also work for negative time interval, hence try h =
  // -0.1
  std::vector<double> h_vals{{0.5, 0.4, 0.1, -0.1}};

  // The solver can get stuck at local minima, so we try many initial guesses.
  // If this test fails to find a solution, append more initial guess to this
  // list.
  std::vector<Eigen::Vector3d> angular_vel_initial_guesses;
  angular_vel_initial_guesses.emplace_back(0, 0, 0);
  // angular_vel = 0 isn't a good initial guess, since the constraint is not
  // really differentiable at angular_vel = 0 (although we have tried to smooth
  // the constraint and its gradient). Hence add another initial guess of
  // angular velocity.
  angular_vel_initial_guesses.emplace_back(1, 0, 0);
  for (const auto& quat1_val : quat1_vals) {
    quat1_cnstr.evaluator()->set_bounds(quat1_val, quat1_val);
    for (const auto& quat2_val : quat2_vals) {
      quat2_cnstr.evaluator()->set_bounds(quat2_val, quat2_val);
      for (const auto& h_val : h_vals) {
        h_cnstr.evaluator()->set_bounds(Vector1d(h_val), Vector1d(h_val));
        solvers::MathematicalProgramResult result;
        for (const auto& angular_vel_init : angular_vel_initial_guesses) {
          prog.SetInitialGuess(angular_vel, angular_vel_init);
          result = solvers::Solve(prog);
          if (result.is_success()) {
            break;
          }
        }
        EXPECT_TRUE(result.is_success());
        // Now compute quat2 using quat1, angular_vel and h, compare that with
        // the solution from the optimization.
        const Eigen::Quaterniond quat2_expected = ComputeNextQuaternion(
            Eigen::Quaterniond(quat1_val(0), quat1_val(1), quat1_val(2),
                               quat1_val(3)),
            result.GetSolution(angular_vel), h_val);
        ExpectTwoQuaternionsClose(
            Eigen::Quaterniond(quat2_val(0), quat2_val(1), quat2_val(2),
                               quat2_val(3)),
            quat2_expected, 1E-5, allow_quaternion_negation);
      }
    }
  }
  // Test the special case that q2 = -q1. Both q2 and q1 represent the same
  // orientation. angular_vel = 0 should be a solution.
  for (const auto& quat1_val : quat1_vals) {
    quat1_cnstr.evaluator()->set_bounds(quat1_val, quat1_val);
    quat2_cnstr.evaluator()->set_bounds(-quat1_val, -quat1_val);
    for (const auto h_val : h_vals) {
      h_cnstr.evaluator()->set_bounds(Vector1d(h_val), Vector1d(h_val));
      prog.SetInitialGuess(angular_vel, Eigen::Vector3d(1, 1, 1));
      auto result = solvers::Solve(prog);
      EXPECT_TRUE(result.is_success());
      const Eigen::Quaterniond quat2_expected =
          ComputeNextQuaternion(Eigen::Quaterniond(quat1_val(0), quat1_val(1),
                                                   quat1_val(2), quat1_val(3)),
                                result.GetSolution(angular_vel), h_val);
      ExpectTwoQuaternionsClose(
          Eigen::Quaterniond(-quat1_val(0), -quat1_val(1), -quat1_val(2),
                             -quat1_val(3)),
          quat2_expected, 1E-5, allow_quaternion_negation);
    }
  }
}

GTEST_TEST(QuaternionEulerIntegrationConstraintTest, SolveAngularVel) {
  TestSolveAngularVel(true);
  TestSolveAngularVel(false);
}

}  // namespace multibody
}  // namespace drake
