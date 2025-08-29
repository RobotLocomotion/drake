#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

void ExpectEqual(const SapHolonomicConstraint<double>& c1,
                 const SapHolonomicConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.parameters(), c2.parameters());
  EXPECT_EQ(c1.constraint_function(), c2.constraint_function());
  EXPECT_EQ(c1.bias(), c2.bias());
}

class SapHolonomicConstraintTests : public ::testing::Test {
 public:
  void SetUp() override {
    SapHolonomicConstraint<double>::Parameters parameters =
        MakeArbitraryParameters();
    dut_ = std::make_unique<SapHolonomicConstraint<double>>(
        g_, SapConstraintJacobian<double>{clique1_, J_}, b_,
        std::move(parameters));
  }

  template <typename T = double>
  static typename SapHolonomicConstraint<T>::Parameters MakeArbitraryParameters(
      double beta = 0) {
    VectorX<T> impulse_lower_limits = -Vector3<T>(1.0, 2.0, 3.0);
    VectorX<T> impulse_upper_limits = Vector3<T>(1.0, 2.0, 3.0);
    VectorX<T> stiffnesses = 1.0e4 * Vector3<T>(1.0, 2.0, 3.0);
    VectorX<T> relaxation_times = 0.01 * Vector3<T>(1.0, 2.0, 3.0);
    return typename SapHolonomicConstraint<T>::Parameters(
        std::move(impulse_lower_limits), std::move(impulse_upper_limits),
        std::move(stiffnesses), std::move(relaxation_times), beta);
  }

  // Makes the same arbitrary set of parameters as MakeArbitraryParameters(),
  // but with an infinite stiffness.
  static SapHolonomicConstraint<double>::Parameters
  MakeArbitraryParametersWithInfiniteStiffness(double beta = 0) {
    const SapHolonomicConstraint<double>::Parameters params =
        MakeArbitraryParameters(beta);
    VectorXd stiffnesses =
        Vector3d::Constant(std::numeric_limits<double>::infinity());
    // For a finite value of dissipation c, the dissipation time scale
    // defined as tau_d = c / k (with k the stiffness), is zero in the limit to
    // infinite stiffness.
    VectorXd relaxation_times = Vector3d::Zero();
    return SapHolonomicConstraint<double>::Parameters(
        params.impulse_lower_limits(), params.impulse_upper_limits(),
        std::move(stiffnesses), std::move(relaxation_times), beta);
  }

  // Helper to make an AutoDiffXd version of constraint `c`.
  // N.B. Regardless of the MatrixBlock format in the original constraint `c`,
  // the new constraint will use dense storage for the Jacobian.
  static std::unique_ptr<SapHolonomicConstraint<AutoDiffXd>> ToAutoDiff(
      const SapHolonomicConstraint<double>& c) {
    const SapHolonomicConstraint<double>::Parameters& p = c.parameters();
    SapHolonomicConstraint<AutoDiffXd>::Parameters p_ad(
        p.impulse_lower_limits(), p.impulse_upper_limits(), p.stiffnesses(),
        p.relaxation_times(), p.beta());
    return std::make_unique<SapHolonomicConstraint<AutoDiffXd>>(
        c.constraint_function(),
        SapConstraintJacobian<AutoDiffXd>{
            c.first_clique(), c.first_clique_jacobian().MakeDenseMatrix()},
        c.bias(), p_ad);
  }

 protected:
  int clique1_{12};
  MatrixXd J_{1.5 * MatrixXd::Ones(3, 3)};
  VectorXd g_{VectorXd::LinSpaced(3, 1.0, 3.0)};
  VectorXd b_{Vector3d(1.0, 2.0, 3.0)};
  std::unique_ptr<SapHolonomicConstraint<double>> dut_;
};

TEST_F(SapHolonomicConstraintTests, SingleCliqueConstruction) {
  EXPECT_EQ(dut_->num_cliques(), 1);
  EXPECT_EQ(dut_->num_constraint_equations(), 3);

  EXPECT_EQ(dut_->first_clique(), clique1_);
  EXPECT_THROW(dut_->second_clique(), std::exception);
  EXPECT_EQ(dut_->constraint_function(), g_);
  EXPECT_EQ(dut_->first_clique_jacobian().MakeDenseMatrix(), J_);
  EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
  EXPECT_EQ(dut_->bias(), b_);
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
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(
      g, SapConstraintJacobian<double>{clique1, J1, clique2, J2}, p);

  EXPECT_EQ(dut_->num_cliques(), 2);
  EXPECT_EQ(dut_->num_constraint_equations(), 3);

  EXPECT_EQ(dut_->first_clique(), clique1);
  EXPECT_EQ(dut_->second_clique(), clique2);
  EXPECT_EQ(dut_->constraint_function(), g);
  EXPECT_EQ(dut_->first_clique_jacobian().MakeDenseMatrix(), J1);
  EXPECT_EQ(dut_->second_clique_jacobian().MakeDenseMatrix(), J2);
  EXPECT_EQ(dut_->bias(), Vector3d::Zero());
  EXPECT_EQ(dut_->parameters().impulse_lower_limits(),
            p.impulse_lower_limits());
  EXPECT_EQ(dut_->parameters().impulse_upper_limits(),
            p.impulse_upper_limits());
  EXPECT_EQ(dut_->parameters().stiffnesses(), p.stiffnesses());
  EXPECT_EQ(dut_->parameters().relaxation_times(), p.relaxation_times());
  EXPECT_EQ(dut_->parameters().beta(), p.beta());
}

TEST_F(SapHolonomicConstraintTests, ValidateConstraintGradients) {
  std::unique_ptr<SapHolonomicConstraint<AutoDiffXd>> dut_ad =
      ToAutoDiff(*dut_);

  const int ne = dut_->num_constraint_equations();
  const double time_step = 0.02;
  VectorX<AutoDiffXd> delassus_estimation =
      VectorX<AutoDiffXd>::Constant(ne, 1.5);  // Arbitrary value.
  std::unique_ptr<AbstractValue> abstract_data =
      dut_ad->MakeData(time_step, delassus_estimation);
  const auto& data =
      abstract_data->get_value<SapHolonomicConstraintData<AutoDiffXd>>();

  // Helper to make a new vector v such that v < x componentwise.
  auto make_lower_vector = [](const VectorXd& x) {
    VectorXd v(x.size());
    for (int i = 0; i < x.size(); ++i) {
      if (x(i) < 0) {
        v(i) = 1.1 * x(i);
      } else {
        v(i) = 0.9 * x(i);
      }
    }
    return v;
  };

  // Helper to make a new vector v such that v > x componentwise.
  auto make_greater_vector = [](const VectorXd& x) {
    VectorXd v(x.size());
    for (int i = 0; i < x.size(); ++i) {
      if (x(i) < 0) {
        v(i) = 0.9 * x(i);
      } else {
        v(i) = 1.1 * x(i);
      }
    }
    return v;
  };

  // Helper to validate gradients at the constraint velocity vc.
  // It returns impulses at vc.
  auto validate_gradients = [&](const VectorXd& vc) {
    VectorX<AutoDiffXd> gamma_ad(ne);
    VectorX<AutoDiffXd> vc_ad = drake::math::InitializeAutoDiff(vc);
    dut_ad->CalcData(vc_ad, abstract_data.get());
    dut_ad->CalcImpulse(*abstract_data, &gamma_ad);
    const VectorXd gamma = math::ExtractValue(gamma_ad);
    ValidateConstraintGradients(*dut_ad, *abstract_data);
    return gamma;
  };

  const VectorXd R = math::ExtractValue(data.R());
  const VectorXd v_hat = math::ExtractValue(data.v_hat());
  const VectorXd gl = dut_->parameters().impulse_lower_limits();
  const VectorXd gu = dut_->parameters().impulse_upper_limits();
  // Constraint velocity at which gamma = gamma_upper.
  const VectorXd vu = v_hat - R.asDiagonal() * gu;
  // Constraint velocity at which gamma = gamma_lower.
  const VectorXd vl = v_hat - R.asDiagonal() * gl;

  // Impulses within limits.
  {
    const VectorXd vc = 0.3 * vl + 0.7 * vu;
    const VectorXd gamma = validate_gradients(vc);
    // We expect impulses within limits.
    EXPECT_TRUE((gl.array() < gamma.array()).all() &&
                (gamma.array() < gu.array()).all());
  }

  // Impulses below lower limits.
  {
    const VectorXd vc = make_greater_vector(vl);
    const VectorXd gamma = validate_gradients(vc);
    // We expect impulses to be capped at the lower limit.
    EXPECT_TRUE(CompareMatrices(gamma, gl));
  }

  // Impulses above upper limits.
  {
    const VectorXd vc = make_lower_vector(vl);
    const VectorXd gamma = validate_gradients(vc);
    // We expect impulses to be capped at the upper limit.
    EXPECT_TRUE(CompareMatrices(gamma, gu));
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
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(
      g_, SapConstraintJacobian<double>{clique1_, J_}, parameters);

  const double time_step = 0.01;
  const VectorXd delassus_approximation =
      VectorXd::Constant(dut_->num_constraint_equations(), 2.0);
  std::unique_ptr<AbstractValue> abstract_data =
      dut_->MakeData(time_step, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapHolonomicConstraintData<double>>();

  // Near-rigid regularization.
  const VectorXd R_near_rigid =
      beta * beta / (4.0 * M_PI * M_PI) * delassus_approximation;

  // Expected regularization for the provided parameters of compliance.
  const VectorXd& k = dut_->parameters().stiffnesses();
  const VectorXd& tau = dut_->parameters().relaxation_times();
  VectorXd R_expected =
      1.0 / (time_step * (time_step + tau.array()) * k.array());

  // For this case we expect only the stiffer constraint, the third one, to be
  // in the rigid regime.
  R_expected(2) = R_near_rigid(2);

  EXPECT_TRUE(
      CompareMatrices(data.R(), R_expected, kEps, MatrixCompareType::relative));
}

TEST_F(SapHolonomicConstraintTests,
       RegularizationAndBiasFromInfiniteStiffness) {
  // We set a non-zero beta in order to test CalcDiagonalRegularization() for
  // parameters within the near-rigid regime.
  const double beta = 1.5;
  SapHolonomicConstraint<double>::Parameters parameters =
      MakeArbitraryParametersWithInfiniteStiffness(beta);
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(
      g_, SapConstraintJacobian<double>{clique1_, J_}, parameters);

  const double time_step = 0.01;
  const VectorXd delassus_approximation =
      VectorXd::Constant(dut_->num_constraint_equations(), 2.0);
  std::unique_ptr<AbstractValue> abstract_data =
      dut_->MakeData(time_step, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapHolonomicConstraintData<double>>();
  const VectorXd& R = data.R();
  const VectorXd& vhat = data.v_hat();

  // Near-rigid regularization.
  const VectorXd R_near_rigid =
      beta * beta / (4.0 * M_PI * M_PI) * delassus_approximation;

  // Since stiffness is infinite, the expected compliance is R_near_rigid for
  // all components.
  const VectorXd R_expected = R_near_rigid;
  EXPECT_TRUE(
      CompareMatrices(R, R_expected, kEps, MatrixCompareType::relative));

  // Since the stiffness is infinite, the bias term is expected to match the
  // near-rigid regime limit.
  const VectorXd& g0 = dut_->constraint_function();
  const VectorXd vhat_expected = -g0 / (2.0 * time_step);
  EXPECT_TRUE(
      CompareMatrices(vhat, vhat_expected, kEps, MatrixCompareType::relative));
}

TEST_F(SapHolonomicConstraintTests, CalcBiasTerm) {
  // We set a non-zero beta in order to test CalcBiasTerm() for
  // parameters within the near-rigid regime.
  const double beta = 1.5;
  SapHolonomicConstraint<double>::Parameters parameters =
      MakeArbitraryParameters(beta);
  dut_ = std::make_unique<SapHolonomicConstraint<double>>(
      g_, SapConstraintJacobian<double>{clique1_, J_}, b_, parameters);

  const double time_step = 0.01;
  const VectorXd delassus_approximation =
      VectorXd::Constant(dut_->num_constraint_equations(), 2.0);
  std::unique_ptr<AbstractValue> abstract_data =
      dut_->MakeData(time_step, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapHolonomicConstraintData<double>>();
  const VectorXd& vhat = data.v_hat();

  // Expected bias for the provided parameters of compliance.
  const VectorXd& tau = dut_->parameters().relaxation_times();
  const VectorXd& g0 = dut_->constraint_function();
  VectorXd vhat_expected = -g0.array() / (time_step + tau.array()) - b_.array();

  // For this case we expect only the stiffer constraint, the third one, to be
  // in the rigid regime. In the near-rigid regime we set the relaxation time to
  // equal the time step, for a critically damped constraint.
  vhat_expected(2) = -g0(2) / (2.0 * time_step) - b_(2);

  EXPECT_TRUE(
      CompareMatrices(vhat, vhat_expected, kEps, MatrixCompareType::relative));
}

TEST_F(SapHolonomicConstraintTests, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapHolonomicConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(*dut_, *clone);

  // Test ToDouble.
  SapHolonomicConstraint<AutoDiffXd> c_ad(
      g_, SapConstraintJacobian<AutoDiffXd>(clique1_, J_), b_,
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapHolonomicConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(*dut_, *clone_from_ad);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
