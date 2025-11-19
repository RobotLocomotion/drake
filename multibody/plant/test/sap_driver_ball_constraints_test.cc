#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for ball constraints.

  Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHolonomicConstraint;
using drake::systems::Context;
using Eigen::MatrixXd;
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
// ball constraint connecting point P on body A and point Q on body B.
class TwoBodiesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes the model described in the fixture's documentation.
  // @param[in] anchor_bodyA If true, body A will be anchored to the world.
  // Otherwise body A has 6 DOFs as body B does.
  void MakeModel(bool anchor_bodyA) {
    plant_.set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

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

    plant_.AddBallConstraint(*bodyA_, p_AP_, *bodyB_, p_BQ_);

    plant_.Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single ball constraint.
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
  Vector3d p_AP_{1.0, -2.0, 3.0};
  Vector3d p_BQ_{-5.0, 4.0, -6.0};
  const Vector3d kOffset_{0.1, 0.2, 0.3};
};

// This test configures a single ball constraint in variety of ways (causing
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
  plant_.SetFloatingBaseBodyPoseInWorldFrame(context_.get(), *bodyB_,
                                             RigidTransformd(kOffset_));

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a single
  // ball constraint.
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(), 3);

  const auto* constraint = dynamic_cast<const SapHolonomicConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapHolonomicConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // One clique if body A is anchored, two if not.
  const int expected_num_cliques = config.bodyA_anchored ? 1 : 2;
  EXPECT_EQ(constraint->num_cliques(), expected_num_cliques);
  if (expected_num_cliques == 1) {
    EXPECT_EQ(constraint->first_clique(), 1);  // i.e., bodyB's tree
    EXPECT_THROW(constraint->second_clique(), std::exception);
  } else {
    EXPECT_EQ(constraint->first_clique(), 0);
    EXPECT_EQ(constraint->second_clique(), 1);
  }

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 3);
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(),
            Vector3d(-kInfinity, -kInfinity, -kInfinity));
  EXPECT_EQ(p.impulse_upper_limits(),
            Vector3d(kInfinity, kInfinity, kInfinity));

  EXPECT_EQ(p.stiffnesses(), Vector3d(kInfinity, kInfinity, kInfinity));
  EXPECT_EQ(p.relaxation_times(), Vector3d::Zero());

  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a ball constraint is defined as:
  //   g = p_WQ - p_WP
  // We verify its value.
  const VectorXd& g = constraint->constraint_function();
  const RigidTransformd& X_WA = plant_.EvalBodyPoseInWorld(*context_, *bodyA_);
  const RigidTransformd& X_WB = plant_.EvalBodyPoseInWorld(*context_, *bodyB_);
  const Vector3d g_expected = X_WB * p_BQ_ - X_WA * p_AP_;
  EXPECT_TRUE(CompareMatrices(g, g_expected));

  // Verify the constraint Jacobians (based on number of cliques).
  // We know by construction that the 6 generalized velocities for a floating
  // body, A, are laid out in the same order as V_WA. We can express the
  // Jacobian of the constraint with respect to each body's spatial velocity:
  //
  // g = p_PQ_W
  //
  // d(g)/dt = v_PQ_W
  //
  // v_PQ_W = v_WQ - v_WP
  //        = (v_WB + w_WB x p_BQ) - (v_WA + w_WA x p_AP)
  //        = (v_WB - p_BQ x w_WB) - (v_WA - p_AP x w_WA)
  //        = [-[p_BQ]ₓ [I]] ⋅ V_WB - [-[p_AP]ₓ [I]] ⋅ V_WA
  //        = J_B ⋅ V_WB - J_A ⋅ V_WA
  if (expected_num_cliques == 1) {
    const MatrixXd& J = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd J_expected =
        (MatrixXd(3, 6) <<         0,  p_BQ_(2), -p_BQ_(1), 1, 0, 0,
                           -p_BQ_(2),         0,  p_BQ_(0), 0, 1, 0,
                            p_BQ_(1), -p_BQ_(0),         0, 0, 0, 1).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(J, J_expected));
  } else {
    const MatrixXd& Ja = constraint->first_clique_jacobian().MakeDenseMatrix();
    // clang-format off
    const MatrixXd Ja_expected =
      -(MatrixXd(3, 6) <<         0,  p_AP_(2), -p_AP_(1), 1, 0, 0,
                          -p_AP_(2),         0,  p_AP_(0), 0, 1, 0,
                           p_AP_(1), -p_AP_(0),         0, 0, 0, 1).finished();
    // clang-format on
    EXPECT_TRUE(CompareMatrices(Ja, Ja_expected));

    const MatrixXd& Jb = constraint->second_clique_jacobian().MakeDenseMatrix();
    // clang-format off
      const MatrixXd Jb_expected =
        (MatrixXd(3, 6) <<         0,  p_BQ_(2), -p_BQ_(1), 1, 0, 0,
                           -p_BQ_(2),         0,  p_BQ_(0), 0, 1, 0,
                            p_BQ_(1), -p_BQ_(0),         0, 0, 0, 1).finished();
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

INSTANTIATE_TEST_SUITE_P(SapBallConstraintTests, TwoBodiesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

GTEST_TEST(BallConstraintsTests, VerifyIdMapping) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  const Vector3d p_AP(1, 2, 3);
  const Vector3d p_BQ(4, 5, 6);
  MultibodyConstraintId ball_id =
      plant.AddBallConstraint(bodyA, p_AP, bodyB, p_BQ);
  const BallConstraintSpec& ball_spec =
      plant.get_ball_constraint_specs(ball_id);
  EXPECT_EQ(ball_spec.id, ball_id);
  EXPECT_EQ(ball_spec.body_A, bodyA.index());
  EXPECT_EQ(ball_spec.body_B, bodyB.index());
  EXPECT_EQ(ball_spec.p_AP, p_AP);
  EXPECT_EQ(ball_spec.p_BQ, p_BQ);

  const std::map<MultibodyConstraintId, BallConstraintSpec>& ball_specs =
      plant.get_ball_constraint_specs();
  ASSERT_EQ(ssize(ball_specs), 1);

  const MultibodyConstraintId ball_id_from_map = ball_specs.begin()->first;
  const BallConstraintSpec& ball_spec_from_map = ball_specs.begin()->second;

  // Check the id in the map matches the one returned.
  EXPECT_EQ(ball_id, ball_id_from_map);

  // Check that the one spec in the map is equal to `ball_spec`.
  EXPECT_EQ(ball_spec.id, ball_spec_from_map.id);
  EXPECT_EQ(ball_spec.body_A, ball_spec_from_map.body_A);
  EXPECT_EQ(ball_spec.body_B, ball_spec_from_map.body_B);
  EXPECT_EQ(ball_spec.p_AP, ball_spec_from_map.p_AP);
  EXPECT_EQ(ball_spec.p_BQ, ball_spec_from_map.p_BQ);

  // Throw on id to wrong constraint specs type.
  EXPECT_THROW(plant.get_coupler_constraint_specs(ball_id), std::exception);
  EXPECT_THROW(plant.get_weld_constraint_specs(ball_id), std::exception);
}

// Ensure that SAP runs on a constraint specified with p_BQ = std::nullopt.
GTEST_TEST(BallConstraintTests, FinalizedConstraint) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::MakeUnitary());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::MakeUnitary());
  plant.AddBallConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                          /* p_BQ = */ std::nullopt);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  EXPECT_NO_THROW(
      plant.get_contact_results_output_port().Eval<ContactResults<double>>(
          *context));
}

GTEST_TEST(BallConstraintTests, FailOnTAMSI) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(plant.AddBallConstraint(bodyA, Vector3d{0, 0, 0},
                                                      bodyB, Vector3d{0, 0, 0}),
                              ".*TAMSI does not support ball constraints.*");
}

GTEST_TEST(BallConstraintTests, FailOnContinuous) {
  MultibodyPlant<double> plant{0.0};
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddBallConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                              Vector3d{0, 0, 0}),
      ".*Currently ball constraints are only supported for discrete "
      "MultibodyPlant models.*");
}

GTEST_TEST(BallConstraintTests, FailOnFinalized) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddBallConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                              Vector3d{0, 0, 0}),
      ".*Post-finalize calls to 'AddBallConstraint\\(\\)' are not allowed.*");
}

GTEST_TEST(BallConstraintTests, FailOnSameBody) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddBallConstraint(bodyA, Vector3d{0, 0, 0}, bodyA,
                              Vector3d{0, 0, 0}),
      ".*Invalid set of parameters for constraint between bodies 'A' and "
      "'A'.*");
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
