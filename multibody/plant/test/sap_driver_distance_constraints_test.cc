#include <algorithm>
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

/* @file This file tests SapDriver's support for distance constraints.

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

bool operator==(const DistanceConstraintParams& p1,
                const DistanceConstraintParams& p2) {
  if (p1.bodyA() != p2.bodyA()) return false;
  if (p1.bodyB() != p2.bodyB()) return false;
  if (p1.p_AP() != p2.p_AP()) return false;
  if (p1.p_BQ() != p2.p_BQ()) return false;
  if (p1.distance() != p2.distance()) return false;
  if (p1.stiffness() != p2.stiffness()) return false;
  if (p1.damping() != p2.damping()) return false;
  return true;
}

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
  bool hard_constraint{};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const TestConfig& c) {
  out << c.description;
  return out;
}

// Fixture that sets a MultibodyPlant model of two bodies A and B with a
// distance constraint connecting point P on body A and point Q on body B.
class TwoBodiesTest : public ::testing::TestWithParam<TestConfig> {
 public:
  // Makes the model described in the fixture's documentation.
  // @param[in] anchor_bodyA If true, body A will be anchored to the world.
  // Otherwise body A has 6 DOFs as body B does.
  // @param[in] use_hard_constraint_defaults If true, it uses the
  // MultibodyPlant::AddDistanceConstraint() defaults for a hard constraint with
  // no dissipation.
  void MakeModel(bool anchor_bodyA, bool use_hard_constraint_defaults) {
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

    if (use_hard_constraint_defaults) {
      plant_.AddDistanceConstraint(*bodyA_, p_AP_, *bodyB_, p_BQ_, kDistance_);
    } else {
      plant_.AddDistanceConstraint(*bodyA_, p_AP_, *bodyB_, p_BQ_, kDistance_,
                                   kStiffness_, kDissipation_);
    }

    plant_.Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    // Model with a single distance constraint.
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

  // Parameters of the problem.
  Vector3d p_AP_{0.1, 0.0, 0.0};
  Vector3d p_BQ_{-0.05, 0.0, 0.0};
  const double kStiffness_{3.0e4};
  const double kDissipation_{1.5};
  const double kDistance_{1.2};
};

// This test configures a single distance constraint in variety of ways (causing
// differing numbers of cliques and varying constraint properties). It then
// examines the newly added constraint to confirm that its instantiation
// reflects the specification.
TEST_P(TwoBodiesTest, ConfirmConstraintProperties) {
  const TestConfig& config = GetParam();

  MakeModel(config.bodyA_anchored, config.hard_constraint);

  const int expected_num_velocities = config.bodyA_anchored ? 6 : 12;
  EXPECT_EQ(plant_.num_velocities(), expected_num_velocities);

  // Place Bo at known distance from Ao; this does *not* satisfy the
  // constraint. We'll observe a non-zero value when evaluating the constraint
  // function.
  const double kDistanceAoBo = 2.0;
  plant_.SetFloatingBaseBodyPoseInWorldFrame(
      context_.get(), *bodyB_,
      RigidTransformd(Vector3d(kDistanceAoBo, 0.0, 0.0)));

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // Verify the expected number of constraints and equations for a single
  // distance constraint.
  EXPECT_EQ(problem.num_constraints(), 1);
  EXPECT_EQ(problem.num_constraint_equations(), 1);

  const auto* constraint = dynamic_cast<const SapHolonomicConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapHolonomicConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // One clique if body A is anchored, two if not.
  const int expected_num_cliques = config.bodyA_anchored ? 1 : 2;
  EXPECT_EQ(constraint->num_cliques(), expected_num_cliques);
  if (expected_num_cliques == 1) {
    EXPECT_EQ(constraint->first_clique(), 1);  // i.e. body B's tree
    EXPECT_THROW(constraint->second_clique(), std::exception);
  } else {
    EXPECT_EQ(constraint->first_clique(), 0);
    EXPECT_EQ(constraint->second_clique(), 1);
  }

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 1);
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(), Vector1d(-kInfinity));
  EXPECT_EQ(p.impulse_upper_limits(), Vector1d(kInfinity));
  if (config.hard_constraint) {
    EXPECT_EQ(p.stiffnesses(), Vector1d(kInfinity));
    // For a finite dissipation, the dissipation time scale will be zero.
    EXPECT_EQ(p.relaxation_times(), Vector1d(0.0));
  } else {
    EXPECT_EQ(p.stiffnesses(), Vector1d(kStiffness_));
    EXPECT_EQ(p.relaxation_times(), Vector1d(kDissipation_ / kStiffness_));
  }

  // This value is hard-coded in the source. This test serves as a brake to
  // prevent the value changing without notification. Changing this value
  // would lead to a behavior change and shouldn't happen silently.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a distance constraint is defined as:
  //   g = |p_PQ| - kDistance.
  // We verify its value.
  const VectorXd& g = constraint->constraint_function();
  const Vector1d g_expected(kDistanceAoBo - p_AP_.x() + p_BQ_.x() - kDistance_);
  EXPECT_TRUE(CompareMatrices(g, g_expected));

  // Verify the constraint Jacobians (based on number of cliques).
  // We exploit internal knowledge of the fact that the state stores angular
  // velocities first, followed by translational velocities (and the
  // relative positions of the bodies).
  if (expected_num_cliques == 1) {
    const MatrixXd& J = constraint->first_clique_jacobian().MakeDenseMatrix();
    const MatrixXd J_expected = (MatrixXd(1, 6) << 0, 0, 0, 1, 0, 0).finished();
    EXPECT_TRUE(CompareMatrices(J, J_expected));
  } else {
    const MatrixXd& Ja = constraint->first_clique_jacobian().MakeDenseMatrix();
    const MatrixXd Ja_expected =
        (MatrixXd(1, 6) << 0, 0, 0, -1, 0, 0).finished();
    EXPECT_TRUE(CompareMatrices(Ja, Ja_expected));

    const MatrixXd& Jb = constraint->second_clique_jacobian().MakeDenseMatrix();
    const MatrixXd Jb_expected =
        (MatrixXd(1, 6) << 0, 0, 0, 1, 0, 0).finished();
    EXPECT_TRUE(CompareMatrices(Jb, Jb_expected));
  }
}

std::vector<TestConfig> MakeTestCases() {
  return std::vector<TestConfig>{
      {.description = "SingleClique",
       .bodyA_anchored = true,
       .hard_constraint = false},
      {.description = "TwoCliques",
       .bodyA_anchored = false,
       .hard_constraint = false},
      {.description = "SingleCliqueWithDefaultValues",
       .bodyA_anchored = true,
       .hard_constraint = true},
      {.description = "TwoCliquesWithDefaultValues",
       .bodyA_anchored = false,
       .hard_constraint = true},
  };
}

INSTANTIATE_TEST_SUITE_P(SapDistanceConstraintTests, TwoBodiesTest,
                         testing::ValuesIn(MakeTestCases()),
                         testing::PrintToStringParamName());

GTEST_TEST(DistanceConstraintsTests, DistanceConstraintParams) {
  MultibodyPlant<double> plant{0.1};
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyC =
      plant.AddRigidBody("C", SpatialInertia<double>::NaN());
  const Vector3d p_AP(1, 2, 3);
  const Vector3d p_BQ(4, 5, 6);
  const double distance = 1.2;
  const double stiffness = 1.3e7;
  const double damping = 0.5;
  const MultibodyConstraintId distance_id = plant.AddDistanceConstraint(
      bodyA, p_AP, bodyB, p_BQ, distance, stiffness, damping);

  // Add another constraint that is not a distance constraint.
  const MultibodyConstraintId ball_id =
      plant.AddBallConstraint(bodyA, p_AP, bodyB, p_BQ);

  const std::map<MultibodyConstraintId, DistanceConstraintParams>&
      all_default_parms = plant.GetDefaultDistanceConstraintParams();
  const DistanceConstraintParams& default_distance_parms =
      all_default_parms.at(distance_id);

  // Retrieve parameters in the context.
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  {
    const DistanceConstraintParams& p =
        plant.GetDistanceConstraintParams(*context, distance_id);
    EXPECT_EQ(p, default_distance_parms);
    // At this point, the context should store the default parameters.
    const std::map<MultibodyConstraintId, DistanceConstraintParams>& all_parms =
        plant.GetDistanceConstraintParams(*context);
    EXPECT_EQ(all_parms, all_default_parms);
  }

  // Set new parameters in the context.
  const Vector3d new_p_AP = 2.0 * p_AP;
  const Vector3d new_p_BQ = 2.0 * p_AP;
  const double new_distance = 2.0 * distance;
  const double new_stiffness = 2.0 * stiffness;
  const double new_damping = 2.0 * damping;
  // N.B. Updating the constrained bodies is allowed.
  DistanceConstraintParams new_params(bodyB.index(), new_p_AP, bodyC.index(),
                                      new_p_BQ, new_distance, new_stiffness,
                                      new_damping);
  {
    plant.SetDistanceConstraintParams(context.get(), distance_id, new_params);
    const DistanceConstraintParams& p =
        plant.GetDistanceConstraintParams(*context, distance_id);
    EXPECT_EQ(p, new_params);
    // Parameters in the context should now contain the new distance parameters.
    const std::map<MultibodyConstraintId, DistanceConstraintParams>
        updated_params{{distance_id, new_params}};
    const std::map<MultibodyConstraintId, DistanceConstraintParams>& all_parms =
        plant.GetDistanceConstraintParams(*context);
    EXPECT_EQ(all_parms, updated_params);
  }

  // Throws if the id does not correspond to a distance constraint.
  {
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.GetDistanceConstraintParams(*context, ball_id),
        "The constraint id .* does not match any distance constraint "
        "registered with this plant. ");
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.SetDistanceConstraintParams(context.get(), ball_id, new_params),
        "The constraint id .* does not match any distance constraint "
        "registered with this plant. ");
  }

  // Updating to an invalid body throws.
  {
    const BodyIndex invalid_body_index(plant.num_bodies() + 10);
    DistanceConstraintParams invalid_bodyA_params(invalid_body_index, p_AP,
                                                  bodyB.index(), p_BQ, distance,
                                                  stiffness, damping);
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.SetDistanceConstraintParams(context.get(), distance_id,
                                          invalid_bodyA_params),
        "Index .* provided for body A does not correspond to a rigid body in "
        "this MultibodyPlant.");

    DistanceConstraintParams invalid_bodyB_params(bodyA.index(), p_AP,
                                                  invalid_body_index, p_BQ,
                                                  distance, stiffness, damping);
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.SetDistanceConstraintParams(context.get(), distance_id,
                                          invalid_bodyB_params),
        "Index .* provided for body B does not correspond to a rigid body in "
        "this MultibodyPlant.");
  }
}

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(DistanceConstraintTests, FailOnTAMSI) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                                  Vector3d{0, 0, 0}, 1 /* distance */),
      ".*TAMSI does not support distance constraints.*");
}
#pragma GCC diagnostic push

GTEST_TEST(DistanceConstraintTests, FailOnContinuous) {
  MultibodyPlant<double> plant{0.0};
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                              Vector3d{0, 0, 0}, 1 /* distance */);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(plant.EvalTimeDerivatives(*context),
                              ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_body_spatial_accelerations_output_port()
          .Eval<std::vector<SpatialAcceleration<double>>>(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_generalized_acceleration_output_port()
          .Eval<std::vector<SpatialAcceleration<double>>>(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.get_reaction_forces_output_port().Eval(*context),
      ".*continuous.*not.*support.*constraints.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcCenterOfMassTranslationalAccelerationInWorld(*context),
      ".*continuous.*not.*support.*constraints.*");
}

GTEST_TEST(DistanceConstraintTests, FailOnFinalized) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                                  Vector3d{0, 0, 0}, 1 /* distance */),
      ".*Post-finalize calls to 'AddDistanceConstraint\\(\\)' are not "
      "allowed.*");
}

GTEST_TEST(DistanceConstraintTests, FailOnInvalidSpecs) {
  MultibodyPlant<double> plant{0.1};
  plant.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  const RigidBody<double>& bodyA =
      plant.AddRigidBody("A", SpatialInertia<double>::NaN());
  const RigidBody<double>& bodyB =
      plant.AddRigidBody("B", SpatialInertia<double>::NaN());
  // Fail on same body.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyA,
                                  Vector3d{0, 0, 0}, 1 /* distance */),
      "Body indexes are equal.");
  // Fail on distance <= 0.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                                  Vector3d{0, 0, 0}, 0 /* distance */),
      "Distance must be strictly positive.");
  // Fail on negative stiffness.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                                  Vector3d{0, 0, 0}, 1 /* distance */,
                                  -1 /* stiffness */),
      "Stiffness must be strictly positive.");
  // Fail on negative damping.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddDistanceConstraint(bodyA, Vector3d{0, 0, 0}, bodyB,
                                  Vector3d{0, 0, 0}, 1 /* distance */,
                                  1 /* stiffness */, -1 /* damping */),
      "Damping must be positive or zero.");
}

// TODO(amcastro-tri): implement unit tests verifying:
//  - unreasonably small distance between the points

}  // namespace internal
}  // namespace multibody
}  // namespace drake
