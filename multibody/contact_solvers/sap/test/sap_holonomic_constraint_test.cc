#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

class SapHolonomicConstraintTests : public ::testing::Test {
 public:
  void SetUp() override {
    SapHolonomicConstraint<double>::Parameters parameters =
        MakeArbitraryParameters();
    dut_ = std::make_unique<SapHolonomicConstraint<double>>(
        clique1_, g_, J_, std::move(parameters));
  }

  static SapHolonomicConstraint<double>::Parameters MakeArbitraryParameters(
      double beta = 0) {
    VectorXd impulse_lower_limits = -Vector3d(1.0, 2.0, 3.0);
    VectorXd impulse_upper_limits = Vector3d(1.0, 2.0, 3.0);
    VectorXd stiffnesses = 1.0e4 * Vector3d(1.0, 2.0, 3.0);
    VectorXd relaxation_times = 0.01 * Vector3d(1.0, 2.0, 3.0);
    return SapHolonomicConstraint<double>::Parameters(
        std::move(impulse_lower_limits), std::move(impulse_upper_limits),
        std::move(stiffnesses), std::move(relaxation_times), beta);
  }

 protected:
  int clique1_{12};
  MatrixXd J_{1.5 * MatrixXd::Ones(3, 3)};
  VectorXd g_{VectorXd::LinSpaced(3, 1.0, 3.0)};
  std::unique_ptr<SapHolonomicConstraint<double>> dut_;
};

TEST_F(SapHolonomicConstraintTests, SingleCliqueConstruction) {
  EXPECT_EQ(dut_->num_cliques(), 1);
  EXPECT_EQ(dut_->num_constraint_equations(), 3);

  EXPECT_EQ(dut_->first_clique(), clique1_);
  EXPECT_THROW(dut_->second_clique(), std::exception);
  EXPECT_EQ(dut_->constraint_function(), g_);
  EXPECT_EQ(dut_->first_clique_jacobian(), J_);
  EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
  const SapHolonomicConstraint<double>::Parameters p =
      MakeArbitraryParameters();
  EXPECT_EQ(dut_->parameters().impulse_lower_limits(),
            p.impulse_lower_limits());
  EXPECT_EQ(dut_->parameters().impulse_upper_limits(),
            p.impulse_upper_limits());
  EXPECT_EQ(dut_->parameters().stiffnesses(), p.stiffnesses());
  EXPECT_EQ(dut_->parameters().relaxation_times(), p.relaxation_times());
  EXPECT_EQ(dut_->parameters().beta(), p.beta());
}

TEST_F(SapHolonomicConstraintTests, TwoCliquesConstruction) {
  // Make a holonomic constraint between two cliques from an arbitrary set of
  // parameters.
  SapHolonomicConstraint<double>::Parameters p = MakeArbitraryParameters();
  const int clique1 = 12;
  const int clique2 = 34;
  const VectorXd g = g_;
  const MatrixXd J1 = J_;
  const MatrixXd J2 = 1.5 * J_;
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(clique1, clique2, g,
                                                          J1, J2, p);

  EXPECT_EQ(dut_->num_cliques(), 2);
  EXPECT_EQ(dut_->num_constraint_equations(), 3);

  EXPECT_EQ(dut_->first_clique(), clique1);
  EXPECT_EQ(dut_->second_clique(), clique2);
  EXPECT_EQ(dut_->constraint_function(), g);
  EXPECT_EQ(dut_->first_clique_jacobian(), J1);
  EXPECT_EQ(dut_->second_clique_jacobian(), J2);
  EXPECT_EQ(dut_->parameters().impulse_lower_limits(),
            p.impulse_lower_limits());
  EXPECT_EQ(dut_->parameters().impulse_upper_limits(),
            p.impulse_upper_limits());
  EXPECT_EQ(dut_->parameters().stiffnesses(), p.stiffnesses());
  EXPECT_EQ(dut_->parameters().relaxation_times(), p.relaxation_times());
  EXPECT_EQ(dut_->parameters().beta(), p.beta());
}

TEST_F(SapHolonomicConstraintTests, Project) {
  const VectorXd& gl = dut_->parameters().impulse_lower_limits();
  const VectorXd& gu = dut_->parameters().impulse_upper_limits();

  // For this constraint the projection is independent of the regularization R.
  // We test this by setting R to NaN and verifying we still get the expected
  // results.
  const VectorXd R = VectorXd::Constant(dut_->num_constraint_equations(), NAN);

  // Impulses within limits.
  {
    const VectorXd y = 0.3 * gl + 0.7 * gu;
    VectorXd gamma(y.size());
    MatrixXd dPdy;
    dut_->Project(y, R, &gamma, &dPdy);
    EXPECT_TRUE(CompareMatrices(gamma, y, kEps, MatrixCompareType::relative));
    MatrixXd dPdy_expected = MatrixXd::Identity(y.size(), y.size());
    EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected, kEps,
                                MatrixCompareType::relative));
  }

  // Impulses below limits.
  {
    const VectorXd y = 1.5 * gl;
    VectorXd gamma(y.size());
    MatrixXd dPdy;
    dut_->Project(y, R, &gamma, &dPdy);
    EXPECT_TRUE(CompareMatrices(gamma, gl, kEps, MatrixCompareType::relative));
    MatrixXd dPdy_expected = MatrixXd::Zero(y.size(), y.size());
    EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected, kEps,
                                MatrixCompareType::relative));
  }

  // Impulses above limits.
  {
    const VectorXd y = 1.5 * gu;
    VectorXd gamma(y.size());
    MatrixXd dPdy;
    dut_->Project(y, R, &gamma, &dPdy);
    EXPECT_TRUE(CompareMatrices(gamma, gu, kEps, MatrixCompareType::relative));
    MatrixXd dPdy_expected = MatrixXd::Zero(y.size(), y.size());
    EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected, kEps,
                                MatrixCompareType::relative));
  }

  // Arbitrary combination of impulses below/above limits.
  {
    VectorXd y(3);
    y(0) = 1.5 * gu(0);                // above upper limit.
    y(1) = 0.3 * gl(1) + 0.7 * gu(1);  // within limits.
    y(2) = 1.5 * gl(2);                // below lower limit.
    VectorXd gamma(y.size());
    MatrixXd dPdy;
    dut_->Project(y, R, &gamma, &dPdy);
    VectorXd gamma_expected(3);
    gamma_expected(0) = gu(0);
    gamma_expected(1) = y(1);
    gamma_expected(2) = gl(2);
    EXPECT_TRUE(CompareMatrices(gamma, gamma_expected, kEps,
                                MatrixCompareType::relative));
    MatrixXd dPdy_expected = MatrixXd::Zero(y.size(), y.size());
    dPdy_expected(1, 1) = 1.0;
    EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected, kEps,
                                MatrixCompareType::relative));
  }
}

// In this unit test we use the default set of parameters created by the fixture
// but overwrite beta to be non-zero.
TEST_F(SapHolonomicConstraintTests, CalcDiagonalRegularization) {
  // We set a non-zero beta in order to test CalcDiagonalRegularization() for
  // parameters within the near-rigid regime.
  const double beta = 1.5;
  SapHolonomicConstraint<double>::Parameters parameters =
      MakeArbitraryParameters(beta);
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(clique1_, g_, J_,
                                                          parameters);

  const double time_step = 0.01;
  const double delassus_inverse_approximation = 2.0;
  const Vector3d R = dut_->CalcDiagonalRegularization(
      time_step, delassus_inverse_approximation);

  // Near-rigid regularization.
  const double R_near_rigid =
      beta * beta / (4.0 * M_PI * M_PI) * delassus_inverse_approximation;

  // Expected regularization for the provided parameters of compliance.
  const VectorXd& k = dut_->parameters().stiffnesses();
  const VectorXd& tau = dut_->parameters().relaxation_times();
  VectorXd R_expected =
      1.0 / (time_step * (time_step + tau.array()) * k.array());

  // For this case we expect only the stiffer constraint, the third one, to be
  // in the rigid regime.
  R_expected(2) = R_near_rigid;

  EXPECT_TRUE(
      CompareMatrices(R, R_expected, kEps, MatrixCompareType::relative));
}

TEST_F(SapHolonomicConstraintTests, CalcBiasTerm) {
  // We set a non-zero beta in order to test CalcBiasTerm() for
  // parameters within the near-rigid regime.
  const double beta = 1.5;
  SapHolonomicConstraint<double>::Parameters parameters =
      MakeArbitraryParameters(beta);
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(clique1_, g_, J_,
                                                          parameters);

  const double time_step = 0.01;
  const double delassus_inverse_approximation = 2.0;
  const Vector3d vhat =
      dut_->CalcBiasTerm(time_step, delassus_inverse_approximation);

  // Expected bias for the provided parameters of compliance.
  const VectorXd& tau = dut_->parameters().relaxation_times();
  const VectorXd& g0 = dut_->constraint_function();
  VectorXd vhat_expected = -g0.array() / (time_step + tau.array());

  // For this case we expect only the stiffer constraint, the third one, to be
  // in the rigid regime. In the near-rigid regime we set the relaxation time to
  // equal the time step, for a critically damped constraint.
  vhat_expected(2) = -g0(2) / (2.0 * time_step);

  EXPECT_TRUE(
      CompareMatrices(vhat, vhat_expected, kEps, MatrixCompareType::relative));
}

TEST_F(SapHolonomicConstraintTests, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapHolonomicConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_constraint_equations(),
            dut_->num_constraint_equations());
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), clique1_);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->constraint_function(), dut_->constraint_function());
  EXPECT_EQ(clone->first_clique_jacobian(), dut_->first_clique_jacobian());
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
  const SapHolonomicConstraint<double>::Parameters& p = dut_->parameters();
  EXPECT_EQ(clone->parameters().num_constraint_equations(),
            p.num_constraint_equations());
  EXPECT_EQ(clone->parameters().impulse_lower_limits(),
            p.impulse_lower_limits());
  EXPECT_EQ(clone->parameters().impulse_upper_limits(),
            p.impulse_upper_limits());
  EXPECT_EQ(clone->parameters().stiffnesses(), p.stiffnesses());
  EXPECT_EQ(clone->parameters().relaxation_times(), p.relaxation_times());
  EXPECT_EQ(clone->parameters().beta(), p.beta());
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
