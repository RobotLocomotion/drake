#include "drake/multibody/contact_solvers/sap/sap_tendon_constraint.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

using Parameters = SapTendonConstraint<double>::Parameters;
using Kinematics = SapTendonConstraint<double>::Kinematics;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  Parameters p;
  Kinematics k;
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

void ExpectEqual(const SapTendonConstraint<double>& c1,
                 const SapTendonConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.parameters(), c2.parameters());
  EXPECT_EQ(c1.kinematics(), c2.kinematics());
}

template <typename T = double>
Parameters MakeArbitraryParameters() {
  // Arbitrary parameter values.
  const T lower_limit = -1.1;
  const T upper_limit = 2.2;
  const T stiffness = 3.3;
  const T damping = 0.4;
  return Parameters{lower_limit, upper_limit, stiffness, damping};
}

template <typename T = double>
Parameters MakeUpperLimitParameters() {
  // No lower limit, single constraint equation parameters.
  const T lower_limit = -kInf;
  const T upper_limit = 2.2;
  const T stiffness = 3.3;
  const T damping = 0.4;
  return Parameters{lower_limit, upper_limit, stiffness, damping};
}

template <typename T = double>
Parameters MakeLowerLimitParameters() {
  // No upper limit, single constraint equation parameters
  const T lower_limit = -1.1;
  const T upper_limit = kInf;
  const T stiffness = 3.3;
  const T damping = 0.4;
  return Parameters{lower_limit, upper_limit, stiffness, damping};
}

template <typename T = double>
Kinematics MakeArbitrarySingleCliqueKinematics() {
  const int clique0 = 3;
  const int clique0_nv = 4;
  VectorX<T> q0(clique0_nv), a0(clique0_nv);
  q0 << -0.1, 0.2, 0.3, 0.4;
  a0 << 0.5, 0.6, 0.7, -0.8;
  const T offset = 0.9;
  return Kinematics{clique0, q0, a0, offset};
}

template <typename T = double>
Kinematics MakeArbitraryTwoCliqueKinematics() {
  const int clique0 = 3;
  const int clique1 = 12;
  const int clique0_nv = 4;
  const int clique1_nv = 2;
  VectorX<T> q0(clique0_nv), q1(clique1_nv), a0(clique0_nv), a1(clique1_nv);
  q0 << -0.1, 0.2, 0.3, 0.4;
  a0 << 0.5, 0.6, 0.7, -0.8;
  q1 << 0.15, 0.25;
  a1 << 0.35, -0.45;
  const T offset = 0.9;
  return Kinematics{clique0, clique1, q0, q1, a0, a1, offset};
}

class SapTendonConstraintTest : public testing::TestWithParam<TestConfig> {
 public:
  // Makes a SapLimitConstraint from this->GetParam() and an arbitrary clique,
  // clique_dof and clique_nv.
  void SetUp() override {
    const Parameters& p = GetParam().p;
    const Kinematics& k = GetParam().k;
    dut_ = std::make_unique<SapTendonConstraint<double>>(p, k);

    SapTendonConstraint<AutoDiffXd>::Parameters p_ad(
        p.lower_limit(), p.upper_limit(), p.stiffness(), p.damping(), p.beta());
    if (k.num_cliques() == 1) {
      SapTendonConstraint<AutoDiffXd>::Kinematics k_ad(k.clique0(), k.q0(),
                                                       k.a0(), k.offset());
      dut_ad_ = std::make_unique<SapTendonConstraint<AutoDiffXd>>(p_ad, k_ad);
    } else {
      SapTendonConstraint<AutoDiffXd>::Kinematics k_ad(
          k.clique0(), k.clique1(), k.q0(), k.q1(), k.a0(), k.a1(), k.offset());
      dut_ad_ = std::make_unique<SapTendonConstraint<AutoDiffXd>>(p_ad, k_ad);
    }
  }

 protected:
  const double time_step_{2.0e-3};
  std::unique_ptr<SapTendonConstraint<double>> dut_;
  std::unique_ptr<SapTendonConstraint<AutoDiffXd>> dut_ad_;
};

TEST_P(SapTendonConstraintTest, Construction) {
  const Parameters& parameters = GetParam().p;
  const Kinematics& kinematics = GetParam().k;

  const int expected_num_cliques = kinematics.num_cliques();
  const int expected_num_equations = parameters.num_finite_limits();

  EXPECT_EQ(dut_->num_objects(), 0);
  EXPECT_EQ(dut_->first_clique(), kinematics.clique0());
  EXPECT_EQ(dut_->num_velocities(0), kinematics.clique0_nv());
  EXPECT_EQ(dut_->num_constraint_equations(), expected_num_equations);
  EXPECT_EQ(dut_->num_cliques(), expected_num_cliques);

  if (kinematics.num_cliques() == 1) {
    // Throws for a single clique.
    EXPECT_THROW(dut_->second_clique(), std::exception);
    EXPECT_THROW(dut_->num_velocities(1), std::exception);
    EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
  } else {
    EXPECT_EQ(dut_->second_clique(), kinematics.clique1());
    EXPECT_EQ(dut_->num_velocities(1), kinematics.clique1_nv());
  }

  // Value of the constraint function should match value of construction
  // arguments.
  EXPECT_EQ(dut_->constraint_function().size(), expected_num_equations);
  EXPECT_EQ(dut_->constraint_function(),
            SapTendonConstraint<double>::CalcConstraintFunction(parameters,
                                                                kinematics));

  if (kinematics.num_cliques() == 1) {
    // The Jacobian should have dimensions 2 x clique0_nv.
    const MatrixX<double> J = dut_->first_clique_jacobian().MakeDenseMatrix();
    EXPECT_EQ(J.rows(), expected_num_equations);
    EXPECT_EQ(J.cols(), kinematics.clique0_nv());

    // The first row (corresponding to the lower limit equation) should be a0,
    // the second row (corresponding to the upper limit equation) should be -a0.
    if (parameters.has_finite_lower_limit()) {
      EXPECT_EQ(J.topRows(1), kinematics.a0().transpose());
    }
    if (parameters.has_finite_upper_limit()) {
      EXPECT_EQ(J.bottomRows(1), -kinematics.a0().transpose());
    }
  } else {
    // The Jacobian block J0 should have dimensions 2 x clique0_nv.
    // The Jacobian block J1 should have dimensions 2 x clique1_nv.
    const MatrixX<double> J0 = dut_->first_clique_jacobian().MakeDenseMatrix();
    const MatrixX<double> J1 = dut_->second_clique_jacobian().MakeDenseMatrix();
    EXPECT_EQ(J0.rows(), expected_num_equations);
    EXPECT_EQ(J0.cols(), kinematics.clique0_nv());
    EXPECT_EQ(J1.rows(), expected_num_equations);
    EXPECT_EQ(J1.cols(), kinematics.clique1_nv());
    // The first rows (lower limit equation) should be a0 and a1, the second
    // rows (upper limit equation) should be -a0 and -a1.
    if (parameters.has_finite_lower_limit()) {
      EXPECT_EQ(J0.topRows(1), kinematics.a0().transpose());
      EXPECT_EQ(J1.topRows(1), kinematics.a1().transpose());
    }
    if (parameters.has_finite_upper_limit()) {
      EXPECT_EQ(J0.bottomRows(1), -kinematics.a0().transpose());
      EXPECT_EQ(J1.bottomRows(1), -kinematics.a1().transpose());
    }
  }
}

// Unit tests bias term v̂ outside of the near rigid regime.
TEST_P(SapTendonConstraintTest, CalcBiasSoft) {
  const Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very soft
  // to ensure we are in the "soft" regime.
  const double soft_stiffness = 0.5;
  Parameters p(p0.lower_limit(), p0.upper_limit(), soft_stiffness, p0.damping(),
               p0.beta());
  const SapTendonConstraint<double> c(p, GetParam().k);
  const double dissipation_time_scale = p.damping() / p.stiffness();
  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step_, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapTendonConstraintData<double>>();
  const VectorXd v_hat_expected =
      -c.constraint_function() / (time_step_ + dissipation_time_scale);
  EXPECT_TRUE(CompareMatrices(data.invariant_data.v_hat, v_hat_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Unit tests bias term v̂ in the near rigid regime.
TEST_P(SapTendonConstraintTest, CalcBiasNearRigid) {
  const Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very high
  // to ensure we are in the "near-rigid" regime.
  const double near_rigid_stiffness = 1.0e12;
  Parameters p(p0.lower_limit(), p0.upper_limit(), near_rigid_stiffness,
               p0.damping(), p0.beta());
  const SapTendonConstraint<double> c(p, GetParam().k);
  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step_, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapTendonConstraintData<double>>();
  const VectorXd v_hat_expected = -c.constraint_function() / (2 * time_step_);
  EXPECT_TRUE(CompareMatrices(data.invariant_data.v_hat, v_hat_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Unit tests Hessian term H outside of the near rigid regime.
TEST_P(SapTendonConstraintTest, CalcHessianSoft) {
  Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very soft
  // to ensure we are in the "soft" regime.
  const double soft_stiffness = 0.5;
  Parameters p(p0.lower_limit(), p0.upper_limit(), soft_stiffness, p0.damping(),
               p0.beta());
  const SapTendonConstraint<double> c(p, GetParam().k);

  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step_, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapTendonConstraintData<double>>();
  const double dissipation_time_scale = p.damping() / p.stiffness();
  const double Hvalue =
      (time_step_ * (time_step_ + dissipation_time_scale) * p.stiffness());

  const VectorXd H_expected =
      VectorXd::Constant(c.num_constraint_equations(), Hvalue);
  EXPECT_TRUE(CompareMatrices(data.invariant_data.H, H_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Unit tests Hessian term H in the near rigid regime.
TEST_P(SapTendonConstraintTest, CalcHessianNearRigid) {
  Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very high
  // to ensure we are in the "near-rigid" regime.
  const double near_rigid_stiffness = 1.0e12;
  Parameters p(p0.lower_limit(), p0.upper_limit(), near_rigid_stiffness,
               p0.damping(), p0.beta());
  const SapTendonConstraint<double> c(p, GetParam().k);

  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step_, delassus_approximation);
  const auto& data =
      abstract_data->get_value<SapTendonConstraintData<double>>();
  const VectorXd H_expected =
      (p.beta() * p.beta() / (4 * M_PI * M_PI) * delassus_approximation)
          .cwiseInverse();
  EXPECT_TRUE(CompareMatrices(data.invariant_data.H, H_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// This test validates the implementation of analytical constraint gradient and
// Hessian against numerical results obtained with automatic differentiation.
TEST_P(SapTendonConstraintTest, ValidateConstraintGradients) {
  const int num_equations = dut_->num_constraint_equations();
  VectorX<AutoDiffXd> delassus_estimation =
      VectorX<AutoDiffXd>::Constant(num_equations, 1.5);  // Arbitrary value.
  std::unique_ptr<AbstractValue> abstract_data =
      dut_ad_->MakeData(time_step_, delassus_estimation);
  const auto& data =
      abstract_data->get_value<SapTendonConstraintData<AutoDiffXd>>();

  // Helper to make a new vector v such that v < x component-wise.
  auto make_lower_vector = [](const VectorXd& x) {
    VectorXd v(x.size());
    for (int i = 0; i < x.size(); ++i) {
      if (x(i) < 0) {
        v(i) = 1.1 * x(i);
      } else if (x(i) > 0) {
        v(i) = 0.9 * x(i);
      } else {  // v(i) = 0
        v(i) = -1.0;
      }
    }
    return v;
  };

  // Helper to make a new vector v such that v > x component-wise.
  auto make_greater_vector = [](const VectorXd& x) {
    VectorXd v(x.size());
    for (int i = 0; i < x.size(); ++i) {
      if (x(i) < 0) {
        v(i) = 0.9 * x(i);
      } else if (x(i) > 0) {
        v(i) = 1.1 * x(i);
      } else {  // v(i) = 0
        v(i) = 1.0;
      }
    }
    return v;
  };

  // Helper to validate gradients at the constraint velocity vc.
  // It returns impulses and Hessian at vc.
  auto validate_gradients = [&](const VectorXd& vc) {
    VectorX<AutoDiffXd> gamma_ad(num_equations);
    MatrixX<AutoDiffXd> G_ad(num_equations, num_equations);
    VectorX<AutoDiffXd> vc_ad = drake::math::InitializeAutoDiff(vc);
    dut_ad_->CalcData(vc_ad, abstract_data.get());
    dut_ad_->CalcImpulse(*abstract_data, &gamma_ad);
    dut_ad_->CalcCostHessian(*abstract_data, &G_ad);
    const VectorXd gamma = math::ExtractValue(gamma_ad);
    const MatrixXd G = math::ExtractValue(G_ad);
    ValidateConstraintGradients(*dut_ad_, *abstract_data);
    return std::make_pair(gamma, G);
  };

  // Impulses for the limit constraint are computed according to:
  //  γ = H⋅(v̂ - vc)₊
  // Therefore the constraint is active when vc < v̂ and not active otherwise.
  const VectorXd& v_hat = math::ExtractValue(data.invariant_data.v_hat);

  // Constraint is active.
  {
    const VectorXd vc = make_lower_vector(v_hat);
    const auto [gamma, G] = validate_gradients(vc);
    // We expect non-zero impulses.
    EXPECT_TRUE((gamma.array() > 0.0).all());
    EXPECT_TRUE((G.diagonal().array() > 0).all());
  }

  // Constraint is not active.
  {
    const VectorXd vc = make_greater_vector(v_hat);
    const auto [gamma, G] = validate_gradients(vc);
    // We expect zero impulses.
    EXPECT_TRUE((gamma.array() == 0.0).all());
    EXPECT_TRUE((G.diagonal().array() == 0).all());
  }
}

TEST_P(SapTendonConstraintTest, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapTendonConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(*dut_, *clone);
}

TEST_P(SapTendonConstraintTest, ToDouble) {
  // Test ToDouble.
  auto clone_from_ad =
      dynamic_pointer_cast<SapTendonConstraint<double>>(dut_ad_->ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(*dut_, *clone_from_ad);
}

TEST_P(SapTendonConstraintTest, AccumulateGeneralizedImpulses) {
  for (int i = 0; i < GetParam().k.num_cliques(); ++i) {
    const MatrixXd J = i == 0
                           ? dut_->first_clique_jacobian().MakeDenseMatrix()
                           : dut_->second_clique_jacobian().MakeDenseMatrix();
    const int nk = dut_->num_constraint_equations();
    const int nv = dut_->num_velocities(i);
    // Arbitrary vector of impulses.
    const VectorXd gamma = VectorXd::LinSpaced(nk, 1.2, 2.5);
    // Arbitrary initial value.
    const VectorXd tau0 = VectorXd::LinSpaced(nv, -5.0, 3.7);
    const VectorXd tau_expected = tau0 + J.transpose() * gamma;
    VectorXd tau = tau0;
    dut_->AccumulateGeneralizedImpulses(i, gamma, &tau);
    EXPECT_TRUE(CompareMatrices(tau, tau_expected,
                                std::numeric_limits<double>::epsilon(),
                                MatrixCompareType::relative));
  }
}

// Generate cases with finite and infinite lower/upper bounds and 1 or 2
// cliques.
std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "SingleClique",
       .p = MakeArbitraryParameters(),
       .k = MakeArbitrarySingleCliqueKinematics()},
      {.description = "TwoClique",
       .p = MakeArbitraryParameters(),
       .k = MakeArbitraryTwoCliqueKinematics()},
      {.description = "LowerOnlySingleClique",
       .p = MakeLowerLimitParameters(),
       .k = MakeArbitrarySingleCliqueKinematics()},
      {.description = "LowerOnlyTwoClique",
       .p = MakeLowerLimitParameters(),
       .k = MakeArbitraryTwoCliqueKinematics()},
      {.description = "UpperOnlySingleClique",
       .p = MakeUpperLimitParameters(),
       .k = MakeArbitrarySingleCliqueKinematics()},
      {.description = "UpperOnlyTwoClique",
       .p = MakeUpperLimitParameters(),
       .k = MakeArbitraryTwoCliqueKinematics()},
  };
}

INSTANTIATE_TEST_SUITE_P(SapTendonConstraintTest, SapTendonConstraintTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());
}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
