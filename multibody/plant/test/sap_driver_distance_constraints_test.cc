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

namespace drake {
namespace multibody {
namespace internal {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Friend class used to provide access to a selection of private functions in
// SapDriver for testing purposes.
class SapDriverTest {
 public:
  static const ContactProblemCache<double>& EvalContactProblemCache(
      const SapDriver<double>& driver, const Context<double>& context) {
    return driver.EvalContactProblemCache(context);
  }
};

// Fixtures that sets a MultibodyPlant model of two bodies A and B with a
// distance constraint connecting point P on body A and point Q on body B.
class TwoBodiesTest : public ::testing::Test {
 public:
  // Makes the model described in the fixture's documentation.
  // if anchor_bodyA = true, body A will be anchored to the world and only B
  // will be free. Otherwise both bodies are free.
  void MakeModel(bool anchor_bodyA) {
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

    plant_.AddDistanceConstraint(*bodyA_, *bodyB_, p_AP_, p_BQ_, kDistance_,
                                 kStiffness_, kDissipation_);

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

// Verify the constraints defined in the SapContactProblem defined by the
// driver, for the case in which only one clique is involved.
TEST_F(TwoBodiesTest, SingleClique) {
  MakeModel(true);
  EXPECT_EQ(plant_.num_velocities(), 6);

  // Place Bo at known distance from Ao.
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

  const auto* constraint = dynamic_cast<const SapHolonomicConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapHolonomicConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // There is a single clique in this problem for body B.
  EXPECT_EQ(constraint->num_cliques(), 1);
  EXPECT_EQ(constraint->first_clique(), 0);
  EXPECT_THROW(constraint->second_clique(), std::exception);

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 1);
  const double kInfinity = std::numeric_limits<double>::infinity();
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(), Vector1d(-kInfinity));
  EXPECT_EQ(p.impulse_upper_limits(), Vector1d(kInfinity));
  EXPECT_EQ(p.stiffnesses(), Vector1d(kStiffness_));
  EXPECT_EQ(p.relaxation_times(), Vector1d(kDissipation_ / kStiffness_));

  // This value is hard-coded in the source. This unit test simply verifies it
  // does not go out of sync.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a distance constraint is defined as:
  //   g = |p_PQ| - kDistance.
  // We verify its value.
  const VectorXd& g = constraint->constraint_function();
  const Vector1d g_expected(kDistanceAoBo - p_AP_.x() + p_BQ_.x() - kDistance_);
  EXPECT_TRUE(CompareMatrices(g, g_expected));

  // Verify constraint Jacobian.
  const MatrixXd& J = constraint->first_clique_jacobian();

  // We use internal knowledge of the fact that the state stores angular
  // velocities first, followed by translational velocities. Therefore the
  // Jacobian will be:
  const MatrixXd J_expected = (MatrixXd(1, 6) << 0, 0, 0, 1, 0, 0).finished();
  EXPECT_TRUE(CompareMatrices(J, J_expected));
}

// Verify the constraints defined in the SapContactProblem defined by the
// driver, for the case in which two cliques are involved.
TEST_F(TwoBodiesTest, TwoCliques) {
  MakeModel(false);
  EXPECT_EQ(plant_.num_velocities(), 12);

  // Place Bo at known distance from Ao.
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

  const auto* constraint = dynamic_cast<const SapHolonomicConstraint<double>*>(
      &problem.get_constraint(0));
  // Verify it is a SapHolonomicConstraint as expected.
  ASSERT_NE(constraint, nullptr);

  // There is a single clique in this problem for body B.
  EXPECT_EQ(constraint->num_cliques(), 2);
  EXPECT_EQ(constraint->first_clique(), 0);
  EXPECT_EQ(constraint->second_clique(), 1);

  // Verify parameters.
  const SapHolonomicConstraint<double>::Parameters p = constraint->parameters();
  EXPECT_EQ(p.num_constraint_equations(), 1);
  const double kInfinity = std::numeric_limits<double>::infinity();
  // bi-lateral constraint with no impulse bounds.
  EXPECT_EQ(p.impulse_lower_limits(), Vector1d(-kInfinity));
  EXPECT_EQ(p.impulse_upper_limits(), Vector1d(kInfinity));
  EXPECT_EQ(p.stiffnesses(), Vector1d(kStiffness_));
  EXPECT_EQ(p.relaxation_times(), Vector1d(kDissipation_ / kStiffness_));

  // This value is hard-coded in the source. This unit test simply verifies it
  // does not go out of sync.
  EXPECT_EQ(p.beta(), 0.1);

  // The constraint function for a distance constraint is defined as:
  //   g = |p_PQ| - kDistance.
  // We verify its value.
  const VectorXd& g = constraint->constraint_function();
  const Vector1d g_expected(kDistanceAoBo - p_AP_.x() + p_BQ_.x() - kDistance_);
  EXPECT_TRUE(CompareMatrices(g, g_expected));

  // Verify constraint Jacobian.
  const MatrixXd& Ja = constraint->first_clique_jacobian();
  const MatrixXd& Jb = constraint->second_clique_jacobian();

  // We use internal knowledge of the fact that the state stores angular
  // velocities first, followed by translational velocities. Therefore the
  // constraints for each clique will be:
  const MatrixXd Ja_expected = (MatrixXd(1, 6) << 0, 0, 0, -1, 0, 0).finished();
  const MatrixXd Jb_expected = (MatrixXd(1, 6) << 0, 0, 0, 1, 0, 0).finished();
  EXPECT_TRUE(CompareMatrices(Ja, Ja_expected));
  EXPECT_TRUE(CompareMatrices(Jb, Jb_expected));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
