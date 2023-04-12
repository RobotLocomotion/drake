#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

class SapLimitConstraintTest
    : public testing::TestWithParam<SapLimitConstraint<double>::Parameters> {
 public:
  // Makes a SapLimitConstraint from this->GetParam() and an arbitrary clique,
  // clique_dof and clique_nv.
  void SetUp() override {
    dut_ = std::make_unique<SapLimitConstraint<double>>(
        clique_, clique_dof_, clique_nv_, q0_, GetParam());
  }

  // Returns the expected number of equations for the parameters specified for
  // this fixture. For finite lower and upper bounds, we expect the constraint
  // to have two equations, and only one equation when just one of the limits is
  // finite.
  int expected_num_equations() const {
    const SapLimitConstraint<double>::Parameters& p = GetParam();
    int num_equations = 0;
    if (p.lower_limit() > -kInf) ++num_equations;
    if (p.upper_limit() < kInf) ++num_equations;
    return num_equations;
  }

  // The expected constraint function g(q).
  VectorXd MakeExpectedConstraintFunction() const {
    const SapLimitConstraint<double>::Parameters& p = GetParam();
    VectorXd g;
    if (p.lower_limit() == -kInf) {
      // Upper limit only.
      g = Vector1d(p.upper_limit() - q0_);
    } else if (p.upper_limit() == kInf) {
      // Lower limit only.
      g = Vector1d(q0_ - p.lower_limit());
    } else {
      // Both limits.
      g = Vector2d(q0_ - p.lower_limit(), p.upper_limit() - q0_);
    }
    return g;
  }

  // The expected constraint Jacobian J(q).
  MatrixXd MakeExpectedJacobian() const {
    const SapLimitConstraint<double>::Parameters& p = GetParam();
    MatrixXd J;
    if (p.lower_limit() == -kInf) {
      // Upper limit only.
      J = -VectorXd::Unit(clique_nv_, clique_dof_).transpose();
    } else if (p.upper_limit() == kInf) {
      // Lower limit only.
      J = VectorXd::Unit(clique_nv_, clique_dof_).transpose();
    } else {
      // Both limits.
      J = MatrixXd::Zero(2, clique_nv_);
      J(0, clique_dof_) = 1;
      J(1, clique_dof_) = -1;
    }
    return J;
  }

 protected:
  // Arbitrary set of indexes and state.
  const int clique_{12};
  const int clique_dof_{3};
  const int clique_nv_{7};
  const double q0_{3.1};
  std::unique_ptr<SapLimitConstraint<double>> dut_;
};

// Bare minimum sanity checks on a newly constructed limit constraint.
TEST_P(SapLimitConstraintTest, Construction) {
  EXPECT_EQ(dut_->num_constraint_equations(), expected_num_equations());
  EXPECT_EQ(dut_->num_cliques(), 1);
  EXPECT_EQ(dut_->first_clique(), clique_);
  EXPECT_THROW(dut_->second_clique(), std::exception);
  EXPECT_EQ(dut_->constraint_function(), MakeExpectedConstraintFunction());
  EXPECT_EQ(dut_->first_clique_jacobian().MakeDenseMatrix(),
            MakeExpectedJacobian());
  EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
  const SapLimitConstraint<double>::Parameters& p = GetParam();
  EXPECT_EQ(dut_->parameters().lower_limit(), p.lower_limit());
  EXPECT_EQ(dut_->parameters().upper_limit(), p.upper_limit());
  EXPECT_EQ(dut_->parameters().stiffness(), p.stiffness());
  EXPECT_EQ(dut_->parameters().dissipation_time_scale(),
            p.dissipation_time_scale());
  EXPECT_EQ(dut_->parameters().beta(), p.beta());
}

// Unit tests SapLimitConstraintTest::CalcBias().
TEST_P(SapLimitConstraintTest, CalcBias) {
  const double dissipation_time_scale = GetParam().dissipation_time_scale();
  const double time_step = 5e-3;
  const double delassus_approximation = NAN;  // Does not participate.
  const VectorXd vhat = dut_->CalcBiasTerm(time_step, delassus_approximation);
  const VectorXd vhat_expected =
      -dut_->constraint_function() / (time_step + dissipation_time_scale);
  EXPECT_TRUE(CompareMatrices(vhat, vhat_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Unit tests SapLimitConstraintTest::CalcRegularizationSoft() in the "soft"
// regime, as defined in [Castro et al., 2021].
// This test uses the constraint parameters supplied by GetParam() to create a
// new set of parameters where only the stiffness is modified so that the
// constraint is in the "soft" regime. The size of the constraint (1 or 2)
// still depends on the upper/lower limits supplied through GetParam().
TEST_P(SapLimitConstraintTest, CalcRegularizationSoft) {
  SapLimitConstraint<double>::Parameters p0 = GetParam();
  // We make a copy of the parameters and change the stiffness to be very soft
  // to ensure we are in the "soft" regime.
  const double soft_stiffness = 0.5;
  SapLimitConstraint<double>::Parameters p(
      p0.lower_limit(), p0.upper_limit(), soft_stiffness,
      p0.dissipation_time_scale(), p0.beta());
  const SapLimitConstraint<double> c(clique_, clique_dof_, clique_nv_, q0_, p);

  const double time_step = 5e-3;
  const double delassus_approximation = 1.5;
  const VectorXd R =
      c.CalcDiagonalRegularization(time_step, delassus_approximation);

  const double Rvalue =
      1. /
      (time_step * (time_step + p.dissipation_time_scale()) * p.stiffness());

  const VectorXd R_expected =
      VectorXd::Constant(expected_num_equations(), Rvalue);
  EXPECT_TRUE(CompareMatrices(R, R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Unit tests SapLimitConstraintTest::CalcRegularizationSoft() in the
// "near-rigid" regime, as defined in [Castro et al., 2021].
// This test uses the constraint parameters supplied by GetParam() to create a
// new set of parameters where only the stiffness is modified so that the
// constraint is in the "near-rigid" regime. The size of the constraint (1 or 2)
// still depends on the upper/lower limits supplied through GetParam().
TEST_P(SapLimitConstraintTest, CalcRegularizationNearRigid) {
  SapLimitConstraint<double>::Parameters p0 = GetParam();
  // We make a copy of the parameters and change the stiffness to be very high
  // to ensure we are in the "near-rigid" regime.
  const double near_rigid_stiffness = 1.0e12;
  SapLimitConstraint<double>::Parameters p(
      p0.lower_limit(), p0.upper_limit(), near_rigid_stiffness,
      p0.dissipation_time_scale(), p0.beta());
  const SapLimitConstraint<double> c(clique_, clique_dof_, clique_nv_, q0_, p);

  const double time_step = 5e-3;
  const double delassus_approximation = 1.5;
  const VectorXd R =
      c.CalcDiagonalRegularization(time_step, delassus_approximation);

  const double Rvalue =
      p.beta() * p.beta() / (4 * M_PI * M_PI) * delassus_approximation;

  const VectorXd R_expected =
      VectorXd::Constant(expected_num_equations(), Rvalue);
  EXPECT_TRUE(CompareMatrices(R, R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Test projection when impulses are positive.
TEST_P(SapLimitConstraintTest, ProjectPositiveImpulses) {
  const int ne = expected_num_equations();
  // Result should not depend on R.
  const VectorXd R = VectorXd::Constant(ne, NAN);

  // Positive impulses.
  const VectorXd y = VectorXd::LinSpaced(ne, 1.2, 3.4);
  VectorXd gamma;
  gamma.resize(ne);
  MatrixXd dPdy;
  dut_->Project(y, R, &gamma, &dPdy);
  EXPECT_TRUE(CompareMatrices(gamma, y, std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(dPdy, MatrixXd::Identity(ne, ne),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Test projection when impulses are negative.
TEST_P(SapLimitConstraintTest, ProjectNegativeImpulses) {
  const int ne = expected_num_equations();
  // Result should not depend on R.
  const VectorXd R = VectorXd::Constant(ne, NAN);

  // Negative impulses.
  const VectorXd y = VectorXd::LinSpaced(ne, -5.2, -1.1);
  VectorXd gamma;
  gamma.resize(ne);
  MatrixXd dPdy;
  dut_->Project(y, R, &gamma, &dPdy);
  EXPECT_TRUE(CompareMatrices(gamma, VectorXd::Zero(ne),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(dPdy, MatrixXd::Zero(ne, ne),
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Test projection when lower limit impulse (if constrained) is positive and
// upper lower impulse (if constrained) is negative.
TEST_P(SapLimitConstraintTest, PositiveLowerImpulseAndNegativeUpperImpulse) {
  const SapLimitConstraint<double>::Parameters p = GetParam();
  const int ne = expected_num_equations();
  // Result should not depend on R.
  const VectorXd R = VectorXd::Constant(ne, NAN);

  VectorXd y = VectorXd::Zero(ne);
  int i = 0;
  if (p.lower_limit() != -kInf) y(i++) = 1.3;  // Arbitrary positive impulse.
  if (p.upper_limit() != kInf) y(i++) = -0.2;  // Arbitrary negative impulse.
  ASSERT_EQ(i, ne);                            // Sanity check.

  // Expected values:
  // For the upper limit (if != kInf) we know that projection and its derivative
  // will be zero . Therefore we only need to setup the nonzero values for the
  // lower limit.
  VectorXd gamma_expected = VectorXd::Zero(ne);
  MatrixXd dPdy_expected = MatrixXd::Zero(ne, ne);
  if (p.lower_limit() != -kInf) {
    gamma_expected(0) = 1.3;
    dPdy_expected(0, 0) = 1.0;
  }

  VectorXd gamma;
  gamma.resize(ne);
  MatrixXd dPdy;
  dut_->Project(y, R, &gamma, &dPdy);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

// Test projection when lower limit impulse (if constrained) is negative and
// upper lower impulse (if constrained) is positive.
TEST_P(SapLimitConstraintTest, NegativeLowerImpulseAndPositiveUpperImpulse) {
  const SapLimitConstraint<double>::Parameters p = GetParam();
  const int ne = expected_num_equations();
  // Result should not depend on R.
  const VectorXd R = VectorXd::Constant(ne, NAN);

  VectorXd y = VectorXd::Zero(ne);
  int i = 0;
  if (p.lower_limit() != -kInf) y(i++) = -0.2;  // Arbitrary negative impulse.
  if (p.upper_limit() != kInf) y(i++) = 1.3;    // Arbitrary positive impulse.
  ASSERT_EQ(i, ne);                             // Sanity check.

  // Expected values:
  // For the lower limit (if != -kInf) we know that projection and its
  // derivative will be zero . Therefore we only need to setup the nonzero
  // values for the upper limit.
  VectorXd gamma_expected = VectorXd::Zero(ne);
  MatrixXd dPdy_expected = MatrixXd::Zero(ne, ne);
  if (p.upper_limit() != kInf) {
    gamma_expected(ne - 1) = 1.3;
    dPdy_expected(ne - 1, ne - 1) = 1.0;
  }

  VectorXd gamma;
  gamma.resize(ne);
  MatrixXd dPdy;
  dut_->Project(y, R, &gamma, &dPdy);
  EXPECT_TRUE(CompareMatrices(gamma, gamma_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(dPdy, dPdy_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::absolute));
}

TEST_P(SapLimitConstraintTest, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapLimitConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_constraint_equations(), expected_num_equations());
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), clique_);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->constraint_function(), dut_->constraint_function());
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(),
            dut_->first_clique_jacobian().MakeDenseMatrix());
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
  const SapLimitConstraint<double>::Parameters p = GetParam();
  EXPECT_EQ(clone->parameters().lower_limit(), p.lower_limit());
  EXPECT_EQ(clone->parameters().upper_limit(), p.upper_limit());
  EXPECT_EQ(clone->parameters().stiffness(), p.stiffness());
  EXPECT_EQ(clone->parameters().dissipation_time_scale(),
            p.dissipation_time_scale());
  EXPECT_EQ(clone->parameters().beta(), p.beta());
}

const SapLimitConstraint<double>::Parameters lower_only{0.5, kInf, 1.0e5, 0.01,
                                                        0.5};
const SapLimitConstraint<double>::Parameters upper_only{-kInf, 2.3, 1.0e5, 0.01,
                                                        0.5};
const SapLimitConstraint<double>::Parameters lower_and_upper{-0.3, 2.3, 1.0e5,
                                                             0.01, 0.5};

INSTANTIATE_TEST_SUITE_P(SapLimitConstraintTest, SapLimitConstraintTest,
                         testing::Values(lower_only, upper_only,
                                         lower_and_upper));

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
