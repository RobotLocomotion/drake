#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"

#include <algorithm>
#include <array>
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
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/expect_equal.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"
#include "drake/solvers/constraint.h"

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

void ExpectEqual(const SapHuntCrossleyConstraint<double>& c1,
                 const SapHuntCrossleyConstraint<double>& c2) {
  ExpectBaseIsEqual(c1, c2);

  // SapHuntCrossleyConstraint specific.
  EXPECT_EQ(c1.parameters(), c2.parameters());
  EXPECT_EQ(c1.configuration(), c2.configuration());
}

namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J32 is of size 3x2.
// clang-format off
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();

const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

template <typename T = double>
ContactConfiguration<T> MakeArbitraryConfiguration() {
  return ContactConfiguration<T>{
      .objectA = 12,
      .p_ApC_W = Vector3<T>(1., 2., 3.),
      .objectB = 5,
      .p_BqC_W = Vector3<T>(4., 5., 6.),
      .phi = std::numeric_limits<double>::infinity() /* not used */,
      .vn = -0.1,
      .fe = 15.0,
      .R_WC = RotationMatrix<T>::Identity()};
}

template <typename T = double>
typename SapHuntCrossleyConstraint<T>::Parameters MakeArbitraryParameters() {
  return typename SapHuntCrossleyConstraint<T>::Parameters{
      .friction = 0.5,
      .stiffness = 1.0e5,
      .dissipation = 0.01,
      .stiction_tolerance = 1.0e-4};
}

// Unit tests construction for a constraint involving a single clique. Cliques,
// in the context of the SAP formulation, are defined in SapContactProblem. See
// also CliqueJacobian.
GTEST_TEST(SapHuntCrossleyConstraint, SingleCliqueConstraint) {
  const int clique = 12;
  const ContactConfiguration<double> configuration =
      MakeArbitraryConfiguration();
  SapConstraintJacobian<double> J(clique, J32);
  const SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossleyConstraint<double> c(configuration, std::move(J), parameters);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), configuration.objectA);
  EXPECT_EQ(c.object(1), configuration.objectB);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), clique);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
  EXPECT_EQ(c.parameters(), parameters);
  EXPECT_EQ(c.configuration(), configuration);
}

// Unit tests construction for a constraint involving two cliques. Cliques,
// in the context of the SAP formulation, are defined in SapContactProblem. See
// also CliqueJacobian.
GTEST_TEST(SapHuntCrossleyConstraint, TwoCliquesConstraint) {
  const int clique0 = 12;
  const int clique1 = 13;
  const ContactConfiguration<double> configuration =
      MakeArbitraryConfiguration();
  SapConstraintJacobian<double> J(clique0, J32, clique1, J34);
  const SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossleyConstraint<double> c(configuration, std::move(J), parameters);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), configuration.objectA);
  EXPECT_EQ(c.object(1), configuration.objectB);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), clique0);
  EXPECT_EQ(c.second_clique(), clique1);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J34);
  EXPECT_EQ(c.parameters(), parameters);
  EXPECT_EQ(c.configuration(), configuration);
}

// Unit test the addition of SAP's regularization, controlled by parameter
// SapHuntCrossleyConstraint::Parameters::sigma.
GTEST_TEST(SapHuntCrossleyConstraint, SapRegularization) {
  using std::max;

  const int clique = 12;
  const ContactConfiguration<double> configuration =
      MakeArbitraryConfiguration();
  SapConstraintJacobian<double> J(clique, J32);
  SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();

  const double time_step = 1e-2;
  const double w = 1.0;
  const Vector3d delassus_approximation = Vector3d::Constant(w);

  auto calc_invariant_data =
      [&](const SapHuntCrossleyConstraint<double>::Parameters& p) {
        SapHuntCrossleyConstraint<double> c(configuration, J, p);
        std::unique_ptr<AbstractValue> abstract_data =
            c.MakeData(time_step, delassus_approximation);
        const auto& data =
            abstract_data->get_value<SapHuntCrossleyConstraintData<double>>();
        const SapHuntCrossleyConstraintData<double>::InvariantData
            invariant_data = data.invariant_data;
        return invariant_data;
      };

  // No regularization.
  {
    parameters.sigma = 0.0;
    const double epsilon_soft = calc_invariant_data(parameters).epsilon_soft;
    EXPECT_EQ(epsilon_soft, parameters.stiction_tolerance);
  }

  // Small regularization.
  {
    parameters.sigma = 1.0e-4;
    const double epsilon_soft = calc_invariant_data(parameters).epsilon_soft;
    EXPECT_EQ(epsilon_soft, parameters.stiction_tolerance);
  }

  // Large regularization.
  {
    parameters.sigma = 1.0e-2;
    // If all components are equal, then wt = wn = w.
    const double Rt = w * parameters.sigma;
    // We first verify the value of n0.
    const double d = parameters.dissipation;
    const double mu = parameters.friction;
    const double fe0 = configuration.fe;
    const double xdot0 = -configuration.vn;
    // Expected values according to the H&C model, at the previous time step.
    const double f0_expected = fe0 * max(0.0, 1.0 + d * xdot0);  // Force.
    const double n0_expected = f0_expected * time_step;          // Impulse.
    // SAP's effective stiction tolerance.
    const double sap_stiction_tolerance = mu * Rt * n0_expected;
    const SapHuntCrossleyConstraintData<double>::InvariantData invariant_data =
        calc_invariant_data(parameters);
    // For large values of sigma, the stiction tolerances matches that of SAP.
    EXPECT_EQ(invariant_data.n0, n0_expected);
    EXPECT_EQ(invariant_data.epsilon_soft, sap_stiction_tolerance);
  }
}

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // The model approximation used by SapHuntCrossleyConstraint.
  SapHuntCrossleyApproximation approximation{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// Number of arbitrary velocity values generated by
// SapHuntCrossleyConstraintTest for testing.
constexpr int kNumArbitraryValues = 3;

class SapHuntCrossleyConstraintTest
    : public ::testing::TestWithParam<TestConfig> {
 public:
  // This method validates analytical gradients implemented by
  // SapHuntCrossleyConstraint using automatic differentiation.
  void ValidateGradients(const SapHuntCrossleyConstraint<double>::Parameters& p,
                         const Vector3d& vc) const {
    const SapHuntCrossleyApproximation approximation =
        this->GetParam().approximation;

    // Instantiate constraint on AutoDiffXd for automatic differentiation.
    SapHuntCrossleyConstraint<AutoDiffXd>::Parameters p_ad{
        approximation, p.friction,           p.stiffness,
        p.dissipation, p.stiction_tolerance, p.sigma};

    // The Jacobian is irrelevant for this tests. Therefore we set it to
    // garbage.
    const int clique = 0;
    const double infty = std::numeric_limits<double>::infinity();
    SapConstraintJacobian<AutoDiffXd> J(clique,
                                        Matrix3<double>::Constant(infty));

    ContactConfiguration<AutoDiffXd> configuration =
        MakeArbitraryConfiguration<AutoDiffXd>();
    const SapHuntCrossleyConstraint<AutoDiffXd> c(
        std::move(configuration), std::move(J), std::move(p_ad));

    // Verify cost gradients using AutoDiffXd.
    ValidateConstraintGradients(c, vc);
  }

  // Helper to make an arbitrary set of values of normal velocities vn, so that
  // normal impulses are non-zero at the next time step.
  static std::array<double, kNumArbitraryValues>
  MakeArbitraryInContactVelocities() {
    // Negative velocities so that objects move towards each other.
    return {-0.1, -0.15, -.05};
  }

  // Helper to make an arbitrary set of values of normal velocities vn, so that
  // normal impulses are zero at the next time step.
  static std::array<double, kNumArbitraryValues>
  MakeArbitraryNoContactVelocities() {
    // Large enough positive velocities that break contact.
    return {0.1, 0.01, .2};
  }

  // Helper to make an arbitrary set of values of tangential velocities vt, with
  // magnitude much smaller than the `stiction_tolerance` so that we know we are
  // in stiction.
  static std::array<Vector2d, kNumArbitraryValues>
  MakeArbitraryStictionVelocities(double stiction_tolerance) {
    // Use a small enough velocity to ensure we are in the stiction regime.
    const double slip_within_stiction = stiction_tolerance / 100.0;
    return {Vector2d(slip_within_stiction, 0.0),
            Vector2d(slip_within_stiction, -slip_within_stiction),
            Vector2d(-slip_within_stiction, slip_within_stiction)};
  }

  // Helper to make an arbitrary set of values of tangential velocities vt, with
  // magnitude much larger than the `stiction_tolerance` so that we know we are
  // sliding.
  static std::array<Vector2d, kNumArbitraryValues>
  MakeArbitrarySlidingVelocities(double stiction_tolerance) {
    // Use a large enough velocity to ensure we are in the sliding regime.
    const double slip_velocity = 100 * stiction_tolerance;
    return {Vector2d(slip_velocity, 0.0),
            Vector2d(slip_velocity, -slip_velocity),
            Vector2d(-slip_velocity, slip_velocity)};
  }

  // Validates gradients for a constraint with parameters `p` at contact
  // velocies vc formed by combining tangential and normal components, i.e. vc =
  // (vt[i], vn[i]).
  void CombineAndValidateGradients(
      const SapHuntCrossleyConstraint<double>::Parameters& p,
      std::array<Vector2d, kNumArbitraryValues> vt,
      std::array<double, kNumArbitraryValues> vn) const {
    for (int i = 0; i < kNumArbitraryValues; ++i) {
      const Vector3d vc = (Vector3d() << vt[i], vn[i]).finished();
      ValidateGradients(p, vc);
    }
  }
};

// Validate gradients for when ‖vₜ‖ is well below the stiction tolerance.
// That is, the constraint is working in the stiction regime.
TEST_P(SapHuntCrossleyConstraintTest, ValidateGradientsWhenInStiction) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  const auto vn = MakeArbitraryInContactVelocities();
  const auto vt = MakeArbitraryStictionVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt, vn);
}

// Validate gradients for when ‖vₜ‖ is much larger than the stiction tolerance.
// That is, the constraint is working in the sliding regime.
TEST_P(SapHuntCrossleyConstraintTest, ValidateGradientsWhenSliding) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  const auto vn = MakeArbitraryInContactVelocities();
  const auto vt = MakeArbitrarySlidingVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt, vn);
}

// Validate gradients for breaking contact.
TEST_P(SapHuntCrossleyConstraintTest, ValidateGradientsForBreakingContact) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // In stiction.
  const auto vn = MakeArbitraryNoContactVelocities();
  const auto vt_stiction =
      MakeArbitraryStictionVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt_stiction, vn);

  // Sliding.
  const auto vt_sliding = MakeArbitrarySlidingVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt_sliding, vn);
}

// When stiffness and dissipation are zero, the impulse is constant and the
// potential is linear. Verify gradients when the constraint is working in the
// stiction regime.
TEST_P(SapHuntCrossleyConstraintTest,
       InStictionWithZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 0;
  p.dissipation = 0.0;

  const auto vn = MakeArbitraryInContactVelocities();
  const auto vt = MakeArbitraryStictionVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt, vn);
}

// When stiffness and dissipation are zero, the impulse is constant and the
// potential is linear. Verify gradients when the constraint is working in the
// sliding regime.
TEST_P(SapHuntCrossleyConstraintTest,
       SlidingWithZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 1.0e-14;
  p.dissipation = 0.0;

  const auto vn = MakeArbitraryInContactVelocities();
  const auto vt = MakeArbitrarySlidingVelocities(p.stiction_tolerance);
  CombineAndValidateGradients(p, vt, vn);
}

std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "Similar",
       .approximation = SapHuntCrossleyApproximation::kSimilar},
      {.description = "Lagged",
       .approximation = SapHuntCrossleyApproximation::kLagged},
  };
}

INSTANTIATE_TEST_SUITE_P(SapHuntCrossleyConstraintTest,
                         SapHuntCrossleyConstraintTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

GTEST_TEST(SapHuntCrossleyConstraint, SingleCliqueConstraintClone) {
  const int clique = 12;
  const ContactConfiguration<double> configuration =
      MakeArbitraryConfiguration();
  SapConstraintJacobian<double> J(clique, J32);
  const SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossleyConstraint<double> c(configuration, std::move(J), parameters);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone =
      dynamic_pointer_cast<SapHuntCrossleyConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapHuntCrossleyConstraint<AutoDiffXd> c_ad(
      MakeArbitraryConfiguration<AutoDiffXd>(),
      SapConstraintJacobian<AutoDiffXd>(clique, J32),
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapHuntCrossleyConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapHuntCrossleyConstraint, TwoCliquesConstraintClone) {
  const int clique0 = 12;
  const int clique1 = 13;
  const ContactConfiguration<double> configuration =
      MakeArbitraryConfiguration();
  SapConstraintJacobian<double> J(clique0, J32, clique1, J34);
  const SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossleyConstraint<double> c(configuration, std::move(J), parameters);

  auto clone =
      dynamic_pointer_cast<SapHuntCrossleyConstraint<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  ExpectEqual(c, *clone);

  // Test ToDouble.
  SapHuntCrossleyConstraint<AutoDiffXd> c_ad(
      MakeArbitraryConfiguration<AutoDiffXd>(),
      SapConstraintJacobian<AutoDiffXd>(clique0, J32, clique1, J34),
      MakeArbitraryParameters<AutoDiffXd>());
  auto clone_from_ad =
      dynamic_pointer_cast<SapHuntCrossleyConstraint<double>>(c_ad.ToDouble());
  ASSERT_NE(clone_from_ad, nullptr);
  ExpectEqual(c, *clone_from_ad);
}

GTEST_TEST(SapHuntCrossleyConstraint, AccumulateSpatialImpulses) {
  const int clique = 12;
  // Arbitrary orientation of the contact frame C.
  Vector3d axis = Vector3d(1., 1., 1.).normalized();
  math::RotationMatrixd R_WC(AngleAxis<double>(M_PI / 4.0, axis));
  SapConstraintJacobian<double> J(clique, J32);
  ContactConfiguration<double> configuration = MakeArbitraryConfiguration();
  configuration.R_WC = R_WC;
  const SapHuntCrossleyConstraint<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossleyConstraint<double> c(configuration, std::move(J), parameters);

  EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.object(0), configuration.objectA);
  EXPECT_EQ(c.object(1), configuration.objectB);

  // Arbitrary impulse values.
  Vector3d gamma(1., 2., 3.);

  // Expected spatial impulse on B.
  const Vector3d f_B_W = R_WC * gamma;
  const Vector3d t_Bq_W = configuration.p_BqC_W.cross(f_B_W);
  const SpatialForce<double> F_Bq_W(t_Bq_W, f_B_W);

  // Expected spatial impulse on A.
  const Vector3d f_A_W = -f_B_W;
  const Vector3d t_Ap_W = configuration.p_ApC_W.cross(f_A_W);
  const SpatialForce<double> F_Ap_W(t_Ap_W, f_A_W);

  const SpatialForce<double> F0(Vector3d(1., 2., 3), Vector3d(4., 5., 6));
  SpatialForce<double> Faccumulated = F0;  // Initialize to non-zero value.
  SpatialForce<double> F_Bq_W_expected = F0 + F_Bq_W;
  c.AccumulateSpatialImpulses(1, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Bq_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));

  Faccumulated = F0;  // Initialize to non-zero value.
  SpatialForce<double> F_Ap_W_expected = F0 + F_Ap_W;
  c.AccumulateSpatialImpulses(0, gamma, &Faccumulated);
  EXPECT_TRUE(CompareMatrices(
      Faccumulated.get_coeffs(), F_Ap_W_expected.get_coeffs(),
      std::numeric_limits<double>::epsilon(), MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
