#include <algorithm>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for distance constraints. */

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
    plant_.set_discrete_contact_solver(DiscreteContactSolver::kSap);

    // Arbitrary inertia values only used by the driver to build a valid contact
    // problem.
    const double mass = 1.5;
    const double radius = 0.1;
    const SpatialInertia<double> M_Bo =
        SpatialInertia<double>::MakeFromCentralInertia(
            mass, Vector3d::Zero(),
            UnitInertia<double>::SolidSphere(radius) * mass);

    bodyA_ = &plant_.AddRigidBody("A", M_Bo);
    bodyB_ = &plant_.AddRigidBody("B", M_Bo);
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
    plant_.SetFreeBodyPoseInWorldFrame(
        context_.get(), *bodyB_,
        RigidTransformd(Vector3d(kDistanceAoBo, 0.0, 0.0)));

    const ContactProblemCache<double>& problem_cache =
        SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
    const SapContactProblem<double>& problem = *problem_cache.sap_problem;

    // Verify the expected number of constraints and equations for a single
    // distance constraint.
    EXPECT_EQ(problem.num_constraints(), 1);
    EXPECT_EQ(problem.num_constraint_equations(), 1);

    const auto* constraint =
        dynamic_cast<const SapHolonomicConstraint<double>*>(
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
    const SapHolonomicConstraint<double>::Parameters p =
        constraint->parameters();
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
    const Vector1d g_expected(kDistanceAoBo - p_AP_.x() + p_BQ_.x() -
                              kDistance_);
    EXPECT_TRUE(CompareMatrices(g, g_expected));

    // Verify the constraint Jacobians (based on number of cliques).
    // We exploit internal knowledge of the fact that the state stores angular
    // velocities first, followed by translational velocities (and the
    // relative positions of the bodies).
    if (expected_num_cliques == 1) {
      const MatrixXd& J = constraint->first_clique_jacobian();
      const MatrixXd J_expected =
          (MatrixXd(1, 6) << 0, 0, 0, 1, 0, 0).finished();
      EXPECT_TRUE(CompareMatrices(J, J_expected));
    } else {
      const MatrixXd& Ja = constraint->first_clique_jacobian();
      const MatrixXd Ja_expected =
          (MatrixXd(1, 6) << 0, 0, 0, -1, 0, 0).finished();
      EXPECT_TRUE(CompareMatrices(Ja, Ja_expected));

      const MatrixXd& Jb = constraint->second_clique_jacobian();
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

INSTANTIATE_TEST_SUITE_P(
    SapDistanceConstraintTests, TwoBodiesTest,
    testing::ValuesIn(MakeTestCases()),
    testing::PrintToStringParamName());

// TODO(amcastro-tri): implement unit tests verifying:
//  - unreasonably small distance between the points
//  - attempt to use distance constraint with continuous system or TAMSI
//  - illegal values for distance, stiffness, or damping

}  // namespace internal
}  // namespace multibody
}  // namespace drake
