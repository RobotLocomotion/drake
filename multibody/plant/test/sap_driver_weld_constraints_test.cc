#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for weld constraints. */

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

namespace drake {
namespace multibody {
namespace internal {

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }
};

struct TestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  bool bodyA_anchored{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// Fixture that sets a MultibodyPlant model of two bodies A and B with a
// weld constraint connecting point P on body A and point Q on body B.
class TwoBodiesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes the model described in the fixture's documentation.
  // @param[in] anchor_bodyA If true, body A will be anchored to the world.
  // Otherwise body A has 6 DOFs as body B does.
  void MakeModel(bool anchor_bodyA) {
    plant_.set_discrete_contact_solver(DiscreteContactSolver::kSap);

    // Arbitrary inertia values only used by the driver to build a valid contact
    // problem.
    const double mass = 1.5;
    const double radius = 0.1;
    const SpatialInertia<double> M_BBcm =
        SpatialInertia<double>::SolidSphereWithMass(mass, radius);

    bodyA_ = &plant_.AddRigidBody("A", M_BBcm);
    bodyB_ = &plant_.AddRigidBody("B", M_BBcm);
    if (anchor_bodyA) {
      plant_.WeldFrames(plant_.world_frame(), bodyA_->body_frame());
    }

    plant_.AddWeldConstraint(*bodyA_, X_AP_, *bodyB_, X_BQ_);

    plant_.Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single weld constraint.
    EXPECT_EQ(plant_.num_constraints(), 1);

    context_ = plant_.CreateDefaultContext();
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

 protected:
  MultibodyPlant<double> plant_{0.01};  // Discrete model.
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;

  // Parameters of the problem. Arbitrary and non-zero.
  const Vector3d p_AP_{1.0, -2.0, 3.0};
  const Vector3d p_BQ_{-5.0, 4.0, -6.0};
  const RigidTransformd X_AP_{RotationMatrixd::MakeZRotation(0.01*M_PI), p_AP_};
  const RigidTransformd X_BQ_{RotationMatrixd::MakeZRotation(-0.01*M_PI), p_BQ_};
  const Vector3d kOffset_{0.1, 0.2, 0.3};
};

// This test configures a single weld constraint in variety of ways (causing
// differing numbers of cliques and varying constraint properties). It then
// examines the newly added constraint to confirm that its instantiation
// reflects the specification.
TEST_P(TwoBodiesTest, ConfirmConstraintProperties) {
  const TestConfig& config = GetParam();

  MakeModel(config.bodyA_anchored);

  const int expected_num_velocities = config.bodyA_anchored ? 6 : 12;
  EXPECT_EQ(plant_.num_velocities(), expected_num_velocities);

  // Place Bo at known offset from Ao; this does *not* satisfy the
  // constraint. We'll observe a non-zero value when evaluating the constraint
  // function.
  plant_.SetFreeBodyPoseInWorldFrame(context_.get(), *bodyB_,
                                     RigidTransformd(kOffset_));

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a single
  // weld constraint.
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(), 6);

  const auto* constraint = dynamic_cast<const SapHolonomicConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapHolonomicConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // One clique if body A is anchored, two if not.
  const int expected_num_cliques = config.bodyA_anchored ? 1 : 2;
  EXPECT_EQ(constraint->num_cliques(), expected_num_cliques);
  EXPECT_EQ(constraint->first_clique(), 0);
  if (expected_num_cliques == 1) {
    EXPECT_THROW(constraint->second_clique(), std::exception);
  } else {
    EXPECT_EQ(constraint->second_clique(), 1);
  }

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 6);
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(), -kInfinity * Vector6d::Ones());
  EXPECT_EQ(p.impulse_upper_limits(),  kInfinity * Vector6d::Ones());

  EXPECT_EQ(p.stiffnesses(), kInfinity * Vector6d::Ones());
  EXPECT_EQ(p.relaxation_times(), plant_.time_step() * Vector6d::Ones());

  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a weld constraint is defined as:
  //   g = (a_N, p_PQ_N)
  //
  // Where frame N is defined as an interpolated frame "halfway" between frames
  // P and Q. The position of the origin of frame N in the world frame, p_WNo,
  // is the midpoint of p_WPo and p_WQo and its orientation in the world R_WN
  // frame is the midpoint of the spherical linear interpolation of R_WP and
  // R_WQ.
  //
  // a_N = θ⋅λ̂ is the axis-angle representation of of the relative orientation
  // between frames P and Q, expressed in frame N.

  const Vector6d& g = constraint->constraint_function();
  const RigidTransformd& X_WA = plant_.EvalBodyPoseInWorld(*context_, *bodyA_);
  const RigidTransformd& X_WB = plant_.EvalBodyPoseInWorld(*context_, *bodyB_);
  const RigidTransformd X_WP = X_WA * X_AP_;
  const RigidTransformd X_WQ = X_WB * X_BQ_;

  // Linear interpolation of two poses measured in the common frame M.
  const auto& interpolate = [](const RigidTransformd& X_MA,
                               const RigidTransformd& X_MB, double t) {
    return math::RigidTransformd(
        X_MA.rotation().ToQuaternion().slerp(t, X_MB.rotation().ToQuaternion()),
        (1.0 - t) * X_MA.translation() + t * X_MB.translation());
  };

  const math::RigidTransformd X_WN = interpolate(X_WP, X_WQ, 0.5);

  const math::RigidTransformd X_NP = X_WN.InvertAndCompose(X_WP);
  const math::RigidTransformd X_NQ = X_WN.InvertAndCompose(X_WQ);

  const Vector3d p_PQ_N = X_NQ.translation() - X_NP.translation();
  const RotationMatrixd R_PQ = X_NP.rotation().transpose() * X_NQ.rotation();

  const Matrix3d& R = R_PQ.matrix();
  const Vector3d a_N(0.5 * (R(2, 1) - R(1, 2)),
                     0.5 * (R(0, 2) - R(2, 0)),
                     0.5 * (R(1, 0) - R(0, 1)));
  const Vector6d g_expected = (Vector6d() << a_N, p_PQ_N).finished();
  EXPECT_TRUE(CompareMatrices(g, g_expected));

  // Verify the constraint Jacobians (based on number of cliques).
  // We know by construction that the 6 generalized velocities for a floating
  // body, A, are laid out in the same order as V_WA. We can express the
  // Jacobian of the constraint with respect to each body's spatial velocity:
  //
  // g = (a_N, p_PQ_N)
  //
  // d(g)/dt = V_W_PQ_N  ** See derivation for proof of this. **
  //
  // V_W_PQ_N = V_WQ_N - V_WP_N
  //        = (w_WB_N, v_WB_N + w_WB_N x p_BQ_N) - (w_WA_N, v_WA_N + w_WA_N x p_AP_N)
  //        = (w_WB_N, v_WB_N - p_BQ_N x w_WB_N) - (w_WA_N, v_WA_N - p_AP_N x w_WA_N)
  //        = [   [R_NW]       0 ]            [   [R_NW]       0 ]
  //          [-[p_BQ_N]ₓ [R_NW] ] ⋅ V_WB_W - [-[p_AP_N]ₓ [R_NW] ] ⋅ V_WA_W
  //        = J_B ⋅ V_WB - J_A ⋅ V_WA

  const Matrix3d& R_NW = X_WN.rotation().matrix().transpose();
  const Matrix3d R_NA = X_WN.rotation().InvertAndCompose(X_WA.rotation()).matrix();
  const Matrix3d R_NB = X_WN.rotation().InvertAndCompose(X_WB.rotation()).matrix();
  const Vector3d p_BQ_N = R_NB * X_BQ_.translation();
  const Vector3d p_AP_N = R_NA * X_AP_.translation();
  // clang-format off
  const Matrix3d p_AP_Nx = (Matrix3d() <<          0, -p_AP_N(2),  p_AP_N(1),
                                           p_AP_N(2),          0, -p_AP_N(0),
                                          -p_AP_N(1),  p_AP_N(0),          0).finished();
  const Matrix3d p_BQ_Nx = (Matrix3d() <<          0, -p_BQ_N(2),  p_BQ_N(1),
                                           p_BQ_N(2),          0, -p_BQ_N(0),
                                          -p_BQ_N(1),  p_BQ_N(0),          0).finished();
  // clang-format on
  if (expected_num_cliques == 1) {
    const MatrixXd& J = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd J_expected =
        (MatrixXd(6, 6) <<     R_NW, Matrix3d::Zero(),
                           -p_BQ_Nx,             R_NW).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(J, J_expected));
  } else {
    const MatrixXd& Ja = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
    const MatrixXd Ja_expected =
      -(MatrixXd(6, 6) <<      R_NW, Matrix3d::Zero(),
                           -p_AP_Nx,             R_NW).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(Ja, Ja_expected));

    const MatrixXd& Jb = constraint->second_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd Jb_expected =
        (MatrixXd(6, 6) <<     R_NW, Matrix3d::Zero(),
                           -p_BQ_Nx,             R_NW).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(Jb, Jb_expected));
  }
}

std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "SingleClique", .bodyA_anchored = true},
      {.description = "TwoCliques", .bodyA_anchored = false},
  };
}

INSTANTIATE_TEST_SUITE_P(SapWeldConstraintTests, TwoBodiesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

GTEST_TEST(WeldConstraintTests, FailOnTAMSI) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_solver(DiscreteContactSolver::kTamsi);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>{});
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>{});
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyB,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*TAMSI does not support weld constraints.*");
}

GTEST_TEST(WeldConstraintTests, FailOnContinuous) {
  MultibodyPlant<double> plant{0.0};
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>{});
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>{});
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyB,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*Currently weld constraints are only supported for discrete "
      "MultibodyPlant models.*");
}

GTEST_TEST(WeldConstraintTests, FailOnFinalized) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>{});
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>{});
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyB,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*Post-finalize calls to 'AddWeldConstraint\\(\\)' are not allowed.*");
}

GTEST_TEST(WeldConstraintTests, FailOnSameBody) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>{});
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddWeldConstraint(bodyA, RigidTransformd{Vector3d{0, 0, 0}}, bodyA,
                              RigidTransformd{Vector3d{0, 0, 0}}),
      ".*Invalid set of parameters for constraint between bodies 'A' and "
      "'A'.*");
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
