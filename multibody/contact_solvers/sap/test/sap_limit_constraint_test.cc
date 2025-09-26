#include "drake/multibody/contact_solvers/sap/sap_limit_constraint.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"

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

void ExpectEqual(const SapLimitConstraint<double>& c1,
                 const SapLimitConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);
  EXPECT_EQ(c1.clique_dof(), c2.clique_dof());
  EXPECT_EQ(c1.position(), c2.position());
  EXPECT_EQ(c1.constraint_function(), c2.constraint_function());
}

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  SapLimitConstraint<double>::Parameters p;
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

class SapLimitConstraintTest : public testing::TestWithParam<TestConfig> {
 public:
  // Makes a SapLimitConstraint from this->GetParam() and an arbitrary clique,
  // clique_dof and clique_nv.
  void SetUp() override {
    const SapLimitConstraint<double>::Parameters& p = GetParam().p;
    dut_ = std::make_unique<SapLimitConstraint<double>>(clique_, clique_dof_,
                                                        clique_nv_, q0_, p);

    SapLimitConstraint<AutoDiffXd>::Parameters p_ad(
        p.lower_limit(), p.upper_limit(), p.stiffness(),
        p.dissipation_time_scale(), p.beta());
    dut_ad_ = std::make_unique<SapLimitConstraint<AutoDiffXd>>(
        clique_, clique_dof_, clique_nv_, q0_, p_ad);
  }

  // Returns the expected number of equations for the parameters specified for
  // this fixture. For finite lower and upper bounds, we expect the constraint
  // to have two equations, and only one equation when just one of the limits is
  // finite.
  int expected_num_equations() const {
    const SapLimitConstraint<double>::Parameters& p = GetParam().p;
    int num_equations = 0;
    if (p.lower_limit() > -kInf) ++num_equations;
    if (p.upper_limit() < kInf) ++num_equations;
    return num_equations;
  }

  // The expected constraint function g(q).
  VectorXd MakeExpectedConstraintFunction() const {
    const SapLimitConstraint<double>::Parameters& p = GetParam().p;
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
    const SapLimitConstraint<double>::Parameters& p = GetParam().p;
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
  const double time_step_{2.0e-3};
  std::unique_ptr<SapLimitConstraint<double>> dut_;
  std::unique_ptr<SapLimitConstraint<AutoDiffXd>> dut_ad_;
};

// Bare minimum sanity checks on a newly constructed limit constraint.
TEST_P(SapLimitConstraintTest, Construction) {
  EXPECT_EQ(dut_->num_constraint_equations(), expected_num_equations());
  EXPECT_EQ(dut_->num_cliques(), 1);
  EXPECT_EQ(dut_->first_clique(), clique_);
  EXPECT_THROW(dut_->second_clique(), std::exception);
  EXPECT_EQ(dut_->first_clique_jacobian().MakeDenseMatrix(),
            MakeExpectedJacobian());
  EXPECT_THROW(dut_->second_clique_jacobian(), std::exception);
  const SapLimitConstraint<double>::Parameters& p = GetParam().p;
  EXPECT_EQ(dut_->parameters().lower_limit(), p.lower_limit());
  EXPECT_EQ(dut_->parameters().upper_limit(), p.upper_limit());
  EXPECT_EQ(dut_->parameters().stiffness(), p.stiffness());
  EXPECT_EQ(dut_->parameters().dissipation_time_scale(),
            p.dissipation_time_scale());
  EXPECT_EQ(dut_->parameters().beta(), p.beta());
}

// Unit tests SapLimitConstraintTest::CalcBias().
TEST_P(SapLimitConstraintTest, CalcBias) {
  const double dissipation_time_scale = GetParam().p.dissipation_time_scale();
  const double time_step = 5e-3;
  const VectorXd delassus_approximation = VectorXd::Constant(
      dut_->num_constraint_equations(), NAN);  // Does not participate.
  std::unique_ptr<AbstractValue> abstract_data =
      dut_->MakeData(time_step, delassus_approximation);
  const auto& data = abstract_data->get_value<SapLimitConstraintData<double>>();
  const VectorXd v_hat_expected =
      -MakeExpectedConstraintFunction() / (time_step + dissipation_time_scale);
  EXPECT_TRUE(CompareMatrices(data.v_hat(), v_hat_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// This test uses the constraint parameters supplied by GetParam() to create a
// new set of parameters where only the stiffness is modified so that the
// constraint is in the "soft" regime. The size of the constraint (1 or 2)
// still depends on the upper/lower limits supplied through GetParam().
TEST_P(SapLimitConstraintTest, CalcRegularizationSoft) {
  SapLimitConstraint<double>::Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very soft
  // to ensure we are in the "soft" regime.
  const double soft_stiffness = 0.5;
  SapLimitConstraint<double>::Parameters p(
      p0.lower_limit(), p0.upper_limit(), soft_stiffness,
      p0.dissipation_time_scale(), p0.beta());
  const SapLimitConstraint<double> c(clique_, clique_dof_, clique_nv_, q0_, p);

  const double time_step = 5e-3;
  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step, delassus_approximation);
  const auto& data = abstract_data->get_value<SapLimitConstraintData<double>>();

  const double Rvalue =
      1. /
      (time_step * (time_step + p.dissipation_time_scale()) * p.stiffness());

  const VectorXd R_expected =
      VectorXd::Constant(expected_num_equations(), Rvalue);
  EXPECT_TRUE(CompareMatrices(data.R(), R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// This test uses the constraint parameters supplied by GetParam() to create a
// new set of parameters where only the stiffness is modified so that the
// constraint is in the "near-rigid" regime. The size of the constraint (1 or 2)
// still depends on the upper/lower limits supplied through GetParam().
TEST_P(SapLimitConstraintTest, CalcRegularizationNearRigid) {
  SapLimitConstraint<double>::Parameters p0 = GetParam().p;
  // We make a copy of the parameters and change the stiffness to be very high
  // to ensure we are in the "near-rigid" regime.
  const double near_rigid_stiffness = 1.0e12;
  SapLimitConstraint<double>::Parameters p(
      p0.lower_limit(), p0.upper_limit(), near_rigid_stiffness,
      p0.dissipation_time_scale(), p0.beta());
  const SapLimitConstraint<double> c(clique_, clique_dof_, clique_nv_, q0_, p);

  const double time_step = 5e-3;
  const VectorXd delassus_approximation =
      VectorXd::Constant(c.num_constraint_equations(), 1.5);
  std::unique_ptr<AbstractValue> abstract_data =
      c.MakeData(time_step, delassus_approximation);
  const auto& data = abstract_data->get_value<SapLimitConstraintData<double>>();

  const VectorXd R_expected =
      p.beta() * p.beta() / (4 * M_PI * M_PI) * delassus_approximation;

  EXPECT_TRUE(CompareMatrices(data.R(), R_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// This test validates the implementation of analytical constraint gradient and
// Hessian against numerical results obtained with automatic differentiation.
TEST_P(SapLimitConstraintTest, ValidateConstraintGradients) {
  const int ne = expected_num_equations();
  VectorX<AutoDiffXd> delassus_estimation =
      VectorX<AutoDiffXd>::Constant(ne, 1.5);  // Arbitrary value.
  std::unique_ptr<AbstractValue> abstract_data =
      dut_ad_->MakeData(time_step_, delassus_estimation);
  const auto& data =
      abstract_data->get_value<SapLimitConstraintData<AutoDiffXd>>();

  // Helper to make a new vector v such that v < x componentwise.
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

  // Helper to make a new vector v such that v > x componentwise.
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
  // It returns impulses at vc.
  auto validate_gradients = [&](const VectorXd& vc) {
    VectorX<AutoDiffXd> gamma_ad(ne);
    VectorX<AutoDiffXd> vc_ad = drake::math::InitializeAutoDiff(vc);
    dut_ad_->CalcData(vc_ad, abstract_data.get());
    dut_ad_->CalcImpulse(*abstract_data, &gamma_ad);
    const VectorXd gamma = math::ExtractValue(gamma_ad);
    ValidateConstraintGradients(*dut_ad_, *abstract_data);
    return gamma;
  };

  // Impulses for the limit constraint are computed according to:
  //  γ = (−R⁻¹⋅(vc−v̂))₊
  // Therefore the constraint is active when vc < v̂ and not active otherwise.
  const VectorXd& v_hat = math::ExtractValue(data.v_hat());

  // Constraint is active.
  {
    const VectorXd vc = make_lower_vector(v_hat);
    const VectorXd gamma = validate_gradients(vc);
    // We expect non-zero impulses.
    EXPECT_TRUE((gamma.array() > 0.0).all());
  }

  // Constraint is not active.
  {
    const VectorXd vc = make_greater_vector(v_hat);
    const VectorXd gamma = validate_gradients(vc);
    // We expect zero impulses.
    EXPECT_TRUE((gamma.array() == 0.0).all());
  }
}

TEST_P(SapLimitConstraintTest, Clone) {
  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapLimitConstraint<double>>(dut_->Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(*dut_, *clone);

  // Test ToDouble.
  auto clone_from_ad =
      dynamic_pointer_cast<SapLimitConstraint<double>>(dut_ad_->ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(*dut_, *clone_from_ad);
}

TEST_P(SapLimitConstraintTest, AccumulateGeneralizedImpulses) {
  const MatrixXd J = dut_->first_clique_jacobian().MakeDenseMatrix();
  const int nk = dut_->num_constraint_equations();
  const int nv = dut_->num_velocities(0);
  // Arbitrary vector of impulses.
  const VectorXd gamma = VectorXd::LinSpaced(nk, 1.2, 2.5);
  // Arbitrary initial value.
  const VectorXd tau0 = VectorXd::LinSpaced(nv, -5.0, 3.7);
  const VectorXd tau_expected = tau0 + J.transpose() * gamma;
  VectorXd tau = tau0;
  dut_->AccumulateGeneralizedImpulses(0 /* Only one clique */, gamma, &tau);
  EXPECT_TRUE(CompareMatrices(tau, tau_expected,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

// Generate cases with finite and infinite lower/upper bounds.
std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {
          .description = "LowerOnly",
          .p = {0.5, kInf, 1.0e5, 0.01, 0.5},
      },
      {
          .description = "UpperOnly",
          .p = {-kInf, 2.3, 1.0e5, 0.01, 0.5},
      },
      {
          .description = "LowerAndUpper",
          .p = {-0.5, 2.3, 1.0e5, 0.01, 0.5},
      },
  };
}

INSTANTIATE_TEST_SUITE_P(SapLimitConstraintTest, SapLimitConstraintTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
