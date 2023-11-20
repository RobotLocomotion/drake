#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"
#include "drake/solvers/constraint.h"

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

bool operator==(const ContactConfiguration<double>& c1,
                const ContactConfiguration<double>& c2) {
  if (c1.objectA != c2.objectA) return false;
  if (c1.p_ApC_W != c2.p_ApC_W) return false;
  if (c1.objectB != c2.objectB) return false;
  if (c1.p_BqC_W != c2.p_BqC_W) return false;
  if (c1.phi != c2.phi) return false;
  if (c1.fe != c2.fe) return false;
  if (c1.vn != c2.vn) return false;
  if (!c1.R_WC.IsExactlyEqualTo(c2.R_WC)) return false;
  return true;
}

bool operator==(const SapHuntCrossleyConstraint<double>::Parameters& p1,
                const SapHuntCrossleyConstraint<double>::Parameters& p2) {
  if (p1.friction != p2.friction) return false;
  if (p1.stiffness != p2.stiffness) return false;
  if (p1.dissipation != p2.dissipation) return false;
  if (p1.sigma != p2.sigma) return false;
  if (p1.stiction_tolerance != p2.stiction_tolerance) return false;
  if (p1.model != p2.model) return false;
  return true;
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
  const int objectA = 12;
  Vector3<T> p_AoC_W(1., 2., 3.);
  const int objectB = 5;
  Vector3<T> p_BoC_W(4., 5., 6.);
  const T phi0 = std::numeric_limits<double>::infinity();  // Not used.
  const T fe0 = 15.0;
  const T vn0 = -0.1;
  RotationMatrix<T> R_WC;
  return ContactConfiguration<T>{
      objectA, std::move(p_AoC_W), objectB, std::move(p_BoC_W), phi0, vn0, fe0,
      R_WC};
}

template <typename T = double>
typename SapHuntCrossleyConstraint<T>::Parameters MakeArbitraryParameters() {
  typename SapHuntCrossleyConstraint<T>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 1.0e5;
  p.dissipation = 0.01;
  p.stiction_tolerance = 1.0e-4;
  return p;
}

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

  auto calc_const_data =
      [&](const SapHuntCrossleyConstraint<double>::Parameters& p) {
        SapHuntCrossleyConstraint<double> c(configuration, J, p);
        std::unique_ptr<AbstractValue> abstract_data =
            c.MakeData(time_step, delassus_approximation);
        const auto& data =
            abstract_data->get_value<SapHuntCrossleyConstraintData<double>>();
        const SapHuntCrossleyConstraintData<double>::ConstData const_data =
            data.const_data;
        return const_data;
      };

  // No regularization.
  {
    parameters.sigma = 0.0;
    const double epsilon_soft = calc_const_data(parameters).epsilon_soft;
    EXPECT_EQ(epsilon_soft, parameters.stiction_tolerance);
  }

  // Small regularization.
  {
    parameters.sigma = 1.0e-4;
    const double epsilon_soft = calc_const_data(parameters).epsilon_soft;
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
    const SapHuntCrossleyConstraintData<double>::ConstData const_data =
        calc_const_data(parameters);
    // For large values of sigma, the stiction tolerances matches that of SAP.
    EXPECT_EQ(const_data.n0, n0_expected);
    EXPECT_EQ(const_data.epsilon_soft, sap_stiction_tolerance);
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

class SapHuntCrossleyConstraintTest
    : public ::testing::TestWithParam<TestConfig> {
 public:
  // This method validates analytical gradients implemented by
  // SapHuntCrossleyConstraint using automatic differentiation.
  void ValidateGradients(const SapHuntCrossleyConstraint<double>::Parameters& p,
                         const Vector3d& vc) {
    const SapHuntCrossleyApproximation approximation =
        this->GetParam().approximation;

    // Instantiate constraint on AutoDiffXd for automatic differentiation.
    SapHuntCrossleyConstraint<AutoDiffXd>::Parameters p_ad{
        approximation, p.friction,           p.stiffness,
        p.dissipation, p.stiction_tolerance, p.sigma};

    // The Jacobian is irrelevant for this tests. Therefore we set to garbage.
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
};

// Validate gradients for when ‖vₜ‖ is well below the stiction tolerance.
TEST_P(SapHuntCrossleyConstraintTest, RegionI) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Use a small enough velocity to ensure we are in the stiction regime.
  const double slip_within_stiction = p.stiction_tolerance / 100.0;

  // Below we use an arbitrary set of values so that vc leads to stiction.
  {
    const Vector3d vc(slip_within_stiction, 0, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(slip_within_stiction, 2e-5, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(-slip_within_stiction, 2e-5, -0.05);
    ValidateGradients(p, vc);
  }
}

// Validate gradients for when ‖vₜ‖ is much larger than the stiction tolerance.
TEST_P(SapHuntCrossleyConstraintTest, RegionII) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Below we use an arbitrary set of values so that vc leads to sliding.
  // In particular the norm of the tangential velocity is much larger than
  // p.stiction_tolerance.
  {
    const Vector3d vc(0.1, 0, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(0.1, -0.2, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(0.1, 0.05, 0.0);
    ValidateGradients(p, vc);
  }
}

// Validate gradients for breaking contact.
TEST_P(SapHuntCrossleyConstraintTest, RegionIII) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Below we use an arbitrary set of values so that vc leads to no-contact.
  // In particular, the normal velocity is larger enough to break contact.
  {
    const Vector3d vc(0.1, 0, 0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(1.0e-4, -2.0e-3, 0.01);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(-0.1, 0.1, 0.2);
    ValidateGradients(p, vc);
  }
}

// When stiffness and dissipation are zero, the impulse is constant and the
// potential is linear. Verify gradients in the stiction region.
TEST_P(SapHuntCrossleyConstraintTest, RegionIZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 0;
  p.dissipation = 0.0;

  // Below we use an arbitrary set of values so that vc leads to stiction.
  {
    const Vector3d vc(1e-5, 0, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(1e-5, 2e-5, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(-1e-5, 2e-5, -0.05);
    ValidateGradients(p, vc);
  }
}

// When stiffness and dissipation are zero, the impulse is constant and the
// potential is linear. Verify gradients in the sliding region.
TEST_P(SapHuntCrossleyConstraintTest, RegionIIZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossleyConstraint<double>::Parameters p;
  p.friction = 0.5;
  p.stiffness = 1.0e-14;
  p.dissipation = 0.0;

  // Below we use an arbitrary set of values so that vc leads to sliding.
  {
    const Vector3d vc(0.1, 0, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(0.1, -0.2, -0.1);
    ValidateGradients(p, vc);
  }
  {
    const Vector3d vc(0.1, 0.05, 0.0);
    ValidateGradients(p, vc);
  }
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
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), clique);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
  EXPECT_EQ(clone->parameters(), parameters);
  EXPECT_EQ(clone->configuration(), configuration);
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
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), clique0);
  EXPECT_EQ(clone->second_clique(), clique1);
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(clone->second_clique_jacobian().MakeDenseMatrix(), J34);
  EXPECT_EQ(clone->parameters(), parameters);
  EXPECT_EQ(clone->configuration(), configuration);
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
  Vector3d gamma(0., 0., 1.);

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
