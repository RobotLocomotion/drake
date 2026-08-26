#include "drake/multibody/contact_solvers/sap/sap_joint_friction_constraint.h"

#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

void ExpectEqual(const SapJointFrictionConstraint<double>& c1,
                 const SapJointFrictionConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.clique_dof(), c2.clique_dof());
  EXPECT_EQ(c1.parameters(), c2.parameters());
}

class SapJointFrictionConstraintTest : public ::testing::Test {
 public:
  void SetUp() override {
    dut_ = std::make_unique<SapJointFrictionConstraint<double>>(
        clique_, clique_dof_, clique_nv_, parameters_);
    SapJointFrictionConstraint<AutoDiffXd>::Parameters p_ad{
        parameters_.friction, parameters_.sigma};
    dut_ad_ = std::make_unique<SapJointFrictionConstraint<AutoDiffXd>>(
        clique_, clique_dof_, clique_nv_, p_ad);
  }

  // Expected values from the class documentation.
  double R() const { return parameters_.sigma * delassus_estimation_; }
  double gamma_max() const { return time_step_ * parameters_.friction; }
  // Stiction velocity: constraint velocities of magnitude below this value are
  // in stiction, above it are in sliding.
  double vs() const { return R() * gamma_max(); }

  // Makes data with the fixture's time step and Delassus estimation and
  // computes cost, impulse and Hessian at vc.
  struct Evaluation {
    double cost;
    double gamma;
    double G;
  };
  Evaluation Evaluate(double vc) const {
    std::unique_ptr<AbstractValue> data =
        dut_->MakeData(time_step_, Vector1d(delassus_estimation_));
    dut_->CalcData(Vector1d(vc), data.get());
    Evaluation e;
    e.cost = dut_->CalcCost(*data);
    VectorXd gamma(1);
    dut_->CalcImpulse(*data, &gamma);
    e.gamma = gamma(0);
    MatrixXd G(1, 1);
    dut_->CalcCostHessian(*data, &G);
    e.G = G(0, 0);
    return e;
  }

 protected:
  // Arbitrary set of indexes and parameters.
  const int clique_{12};
  const int clique_dof_{3};
  const int clique_nv_{7};
  const double time_step_{2.0e-3};
  const double delassus_estimation_{1.5};
  const SapJointFrictionConstraint<double>::Parameters parameters_{
      .friction = 2.5, .sigma = 1.0e-2};
  std::unique_ptr<SapJointFrictionConstraint<double>> dut_;
  std::unique_ptr<SapJointFrictionConstraint<AutoDiffXd>> dut_ad_;
};

// Bare minimum sanity checks on a newly constructed constraint.
TEST_F(SapJointFrictionConstraintTest, Construction) {
  EXPECT_EQ(dut_->num_constraint_equations(), 1);
  EXPECT_EQ(dut_->num_cliques(), 1);
  EXPECT_EQ(dut_->first_clique(), clique_);
  EXPECT_THROW(dut_->second_clique(), std::exception);
  EXPECT_EQ(dut_->num_objects(), 0);
  EXPECT_EQ(dut_->clique_dof(), clique_dof_);
  EXPECT_EQ(dut_->parameters(), parameters_);
  const MatrixXd J_expected =
      VectorXd::Unit(clique_nv_, clique_dof_).transpose();
  EXPECT_EQ(dut_->first_clique_jacobian().MakeDenseMatrix(), J_expected);
  EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
}

// Verifies the regularization and impulse bound computed by MakeData().
TEST_F(SapJointFrictionConstraintTest, MakeData) {
  std::unique_ptr<AbstractValue> abstract_data =
      dut_->MakeData(time_step_, Vector1d(delassus_estimation_));
  const auto& data =
      abstract_data->get_value<SapJointFrictionConstraintData<double>>();
  EXPECT_NEAR(data.R(), R(), kEps);
  EXPECT_NEAR(data.R_inv(), 1.0 / R(), kEps);
  EXPECT_NEAR(data.gamma_max(), gamma_max(), kEps);
}

// Verifies cost, impulse and Hessian against the analytical expressions in the
// class documentation, in both the stiction and the sliding regimes.
TEST_F(SapJointFrictionConstraintTest, StictionRegime) {
  for (const double vc : {-0.7 * vs(), 0.0, 0.3 * vs()}) {
    const Evaluation e = Evaluate(vc);
    EXPECT_NEAR(e.cost, vc * vc / (2.0 * R()), kEps);
    // The impulse is computed as -vc⋅R⁻¹, which incurs a few roundoff errors
    // relative to -vc/R.
    EXPECT_NEAR(e.gamma, -vc / R(), 4 * kEps * gamma_max());
    EXPECT_NEAR(e.G, 1.0 / R(), kEps / R());
    // In stiction the impulse is strictly within its bounds.
    EXPECT_LT(std::abs(e.gamma), gamma_max());
  }
}

TEST_F(SapJointFrictionConstraintTest, SlidingRegime) {
  for (const double vc : {-3.5 * vs(), 1.2 * vs()}) {
    const Evaluation e = Evaluate(vc);
    const double sign = vc > 0 ? 1.0 : -1.0;
    EXPECT_NEAR(
        e.cost,
        gamma_max() * std::abs(vc) - 0.5 * R() * gamma_max() * gamma_max(),
        kEps);
    // The impulse saturates at the bound and opposes motion.
    EXPECT_NEAR(e.gamma, -sign * gamma_max(), kEps * gamma_max());
    EXPECT_EQ(e.G, 0.0);
  }
}

// The cost and impulse must be continuous at the transition between stiction
// and sliding, |vc| = vs. The Hessian jumps from 1/R to zero.
TEST_F(SapJointFrictionConstraintTest, ContinuityAtTransition) {
  for (const double sign : {-1.0, 1.0}) {
    const double delta = 1.0e-10 * vs();
    const Evaluation below = Evaluate(sign * (vs() - delta));
    const Evaluation above = Evaluate(sign * (vs() + delta));
    // The cost has a continuous first derivative of magnitude γₘₐₓ at the
    // transition, so across the width 2⋅delta it changes by 2⋅γₘₐₓ⋅delta up
    // to second order terms (delta²/(2R), far below roundoff here).
    EXPECT_NEAR(above.cost - below.cost, 2.0 * gamma_max() * delta,
                16 * kEps * above.cost);
    // The impulse is Lipschitz with constant 1/R.
    EXPECT_NEAR(below.gamma, above.gamma, 2.0 * delta / R());
    EXPECT_NEAR(below.G, 1.0 / R(), kEps / R());
    EXPECT_EQ(above.G, 0.0);
  }
}

// Verifies the principle of maximum dissipation: the impulse always opposes
// the constraint velocity and is bounded by gamma_max.
TEST_F(SapJointFrictionConstraintTest, MaximumDissipation) {
  for (const double vc : VectorXd::LinSpaced(21, -5.0 * vs(), 5.0 * vs())) {
    const Evaluation e = Evaluate(vc);
    EXPECT_LE(e.gamma * vc, 0.0);
    EXPECT_LE(std::abs(e.gamma), gamma_max() * (1.0 + kEps));
  }
}

// This test validates the implementation of analytical constraint gradient and
// Hessian against numerical results obtained with automatic differentiation.
TEST_F(SapJointFrictionConstraintTest, ValidateConstraintGradients) {
  std::unique_ptr<AbstractValue> abstract_data =
      dut_ad_->MakeData(time_step_, Vector1<AutoDiffXd>(delassus_estimation_));

  // Helper to validate gradients at the constraint velocity vc.
  // It returns the impulse at vc.
  auto validate_gradients = [&](double vc) {
    VectorX<AutoDiffXd> gamma_ad(1);
    VectorX<AutoDiffXd> vc_ad = math::InitializeAutoDiff(Vector1d(vc));
    dut_ad_->CalcData(vc_ad, abstract_data.get());
    dut_ad_->CalcImpulse(*abstract_data, &gamma_ad);
    ValidateConstraintGradients(*dut_ad_, *abstract_data);
    return math::ExtractValue(gamma_ad)(0);
  };

  // Stiction.
  {
    const double gamma = validate_gradients(0.4 * vs());
    EXPECT_LT(std::abs(gamma), gamma_max());
    EXPECT_LT(gamma, 0.0);
  }
  // Sliding in the positive direction.
  {
    const double gamma = validate_gradients(2.0 * vs());
    EXPECT_NEAR(gamma, -gamma_max(), kEps * gamma_max());
  }
  // Sliding in the negative direction.
  {
    const double gamma = validate_gradients(-2.0 * vs());
    EXPECT_NEAR(gamma, gamma_max(), kEps * gamma_max());
  }
}

TEST_F(SapJointFrictionConstraintTest, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapJointFrictionConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(*dut_, *clone);

  // Test ToDouble.
  auto clone_from_ad = dynamic_pointer_cast<SapJointFrictionConstraint<double>>(
      dut_ad_->ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(*dut_, *clone_from_ad);
}

TEST_F(SapJointFrictionConstraintTest, AccumulateGeneralizedImpulses) {
  const MatrixXd J = dut_->first_clique_jacobian().MakeDenseMatrix();
  const int nv = dut_->num_velocities(0);
  // Arbitrary impulse.
  const Vector1d gamma(1.7);
  // Arbitrary initial value.
  const VectorXd tau0 = VectorXd::LinSpaced(nv, -5.0, 3.7);
  const VectorXd tau_expected = tau0 + J.transpose() * gamma;
  VectorXd tau = tau0;
  dut_->AccumulateGeneralizedImpulses(0 /* Only one clique */, gamma, &tau);
  EXPECT_TRUE(
      CompareMatrices(tau, tau_expected, kEps, MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
