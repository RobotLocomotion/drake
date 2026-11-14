#include <cmath>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_joint_friction_constraint.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

/* @file This file tests SapDriver's support for joint dry friction. */

using drake::multibody::contact_solvers::internal::ContactSolverResults;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapJointFrictionConstraint;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::VectorXd;

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

namespace {

// Relative tolerance for comparisons against analytical solutions. For the
// single DOF problems below the SAP cost is piecewise quadratic and Newton
// iterations converge to machine precision within a few iterations.
constexpr double kTolerance = 1.0e-10;

// Model parameters.
constexpr double kTimeStep = 1.0e-3;     // [s]
constexpr double kWheelInertia = 0.1;    // [kg⋅m²], about the wheel's axis.
constexpr double kWheelFriction = 0.5;   // [N⋅m]
constexpr double kSliderMass = 2.0;      // [kg]
constexpr double kSliderFriction = 1.0;  // [N]
constexpr double kElbowFriction = 0.3;   // [N⋅m]
constexpr double kLinkLength = 0.5;      // [m]

// Fixture with a model made of three kinematic trees, each with a joint that
// models dry friction:
//   1. A wheel free to spin about the world's z axis. A single revolute DOF.
//   2. A slider free to translate along the world's x axis. A single prismatic
//      DOF.
//   3. A two link arm in the x-y plane with a shoulder and an elbow revolute
//      joint about z. Only the elbow models dry friction. This tree is used to
//      verify the indexing of DOFs within a tree with more than one DOF.
// Gravity is set to zero so that actuation is the only force applied on the
// model besides friction. Therefore the wheel and slider dynamics have simple
// analytical solutions used to verify the implementation.
class SapDriverJointFrictionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<MultibodyPlant<double>>(kTimeStep);
    plant_->mutable_gravity_field().set_gravity_vector(Vector3d::Zero());

    // Bodies with the same rotational inertia about every axis so that the
    // inertia about the joint's axis is known regardless of orientation.
    const SpatialInertia<double> M_wheel(
        1.0, Vector3d::Zero(),
        UnitInertia<double>::TriaxiallySymmetric(kWheelInertia));
    const SpatialInertia<double> M_slider(
        kSliderMass, Vector3d::Zero(),
        UnitInertia<double>::TriaxiallySymmetric(1.0));
    // The arm's inertial properties play no role in these tests.
    const SpatialInertia<double> M_link = SpatialInertia<double>::MakeUnitary();

    const RigidBody<double>& wheel = plant_->AddRigidBody("wheel", M_wheel);
    wheel_ = &plant_->AddJoint<RevoluteJoint>("wheel", plant_->world_body(),
                                              std::nullopt, wheel, std::nullopt,
                                              Vector3d::UnitZ());
    wheel_actuator_ = &plant_->AddJointActuator("wheel", *wheel_);
    plant_->get_mutable_joint(wheel_->index())
        .set_default_dry_friction(kWheelFriction);

    const RigidBody<double>& slider = plant_->AddRigidBody("slider", M_slider);
    slider_ = &plant_->AddJoint<PrismaticJoint>(
        "slider", plant_->world_body(), std::nullopt, slider, std::nullopt,
        Vector3d::UnitX());
    slider_actuator_ = &plant_->AddJointActuator("slider", *slider_);
    plant_->get_mutable_joint(slider_->index())
        .set_default_dry_friction(kSliderFriction);

    const RigidBody<double>& link1 = plant_->AddRigidBody("link1", M_link);
    const RigidBody<double>& link2 = plant_->AddRigidBody("link2", M_link);
    plant_->AddJoint<RevoluteJoint>("shoulder", plant_->world_body(),
                                    std::nullopt, link1, std::nullopt,
                                    Vector3d::UnitZ());
    elbow_ = &plant_->AddJoint<RevoluteJoint>(
        "elbow", link1, math::RigidTransformd(Vector3d(kLinkLength, 0.0, 0.0)),
        link2, std::nullopt, Vector3d::UnitZ());
    plant_->get_mutable_joint(elbow_->index())
        .set_default_dry_friction(kElbowFriction);

    plant_->Finalize();

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    context_ = plant_->CreateDefaultContext();
    SetActuation(0.0, 0.0);
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

  // Returns the joint friction constraints of the problem for the state in
  // `context_`.
  std::vector<const SapJointFrictionConstraint<double>*> EvalConstraints()
      const {
    const SapContactProblem<double>& problem =
        *SapDriverTest::EvalContactProblemCache(sap_driver(), *context_)
             .sap_problem;
    std::vector<const SapJointFrictionConstraint<double>*> constraints;
    for (int i = 0; i < problem.num_constraints(); ++i) {
      const auto* constraint =
          dynamic_cast<const SapJointFrictionConstraint<double>*>(
              &problem.get_constraint(i));
      DRAKE_DEMAND(constraint != nullptr);
      constraints.push_back(constraint);
    }
    return constraints;
  }

  // Fixes the actuation input port so that `wheel_torque` is applied on the
  // wheel and `slider_force` is applied on the slider.
  void SetActuation(double wheel_torque, double slider_force) {
    VectorXd u = VectorXd::Zero(plant_->num_actuated_dofs());
    u(wheel_actuator_->input_start()) = wheel_torque;
    u(slider_actuator_->input_start()) = slider_force;
    plant_->get_actuation_input_port().FixValue(context_.get(), u);
  }

  // Takes one step with the AutoDiffXd version of the plant from the state and
  // actuation in `context_`, and returns the resulting velocities. The
  // AutoDiffXd plant uses its own default parameters.
  VectorXd StepAutoDiffXd() const {
    std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
        systems::System<double>::ToAutoDiffXd(*plant_);
    std::unique_ptr<Context<AutoDiffXd>> context_ad =
        plant_ad->CreateDefaultContext();
    context_ad->SetDiscreteState(VectorX<AutoDiffXd>(
        context_->get_discrete_state_vector().value().cast<AutoDiffXd>()));
    plant_ad->get_actuation_input_port().FixValue(
        context_ad.get(), VectorX<AutoDiffXd>(plant_->get_actuation_input_port()
                                                  .Eval(*context_)
                                                  .cast<AutoDiffXd>()));
    plant_ad->ExecuteForcedEvents(context_ad.get(), /* publish = */ false);
    return math::ExtractValue(plant_ad->GetVelocities(*context_ad));
  }

  // Analytical solution for the velocity of a single DOF with effective inertia
  // `inertia` after one step from rest under a constant applied generalized
  // force `applied_force` above the dry friction bound `friction`. Friction
  // saturates at its bound and opposes motion.
  double ExpectedSlidingVelocityFromRest(double applied_force, double inertia,
                                         double friction) const {
    const double sign = applied_force > 0 ? 1.0 : -1.0;
    return kTimeStep * (applied_force - sign * friction) / inertia;
  }

  // Analytical solution for the velocity of a single DOF with effective inertia
  // `inertia` after one step from rest under a constant applied generalized
  // force `applied_force` below the dry friction bound. In stiction, SAP
  // minimizes the quadratic cost ½⋅m⋅(v − v*)² + v²/(2R), with v* = δt⋅τₐ/m the
  // velocity without friction and R = σ⋅w the regularization of friction, see
  // SapJointFrictionConstraint. For a tree with a single DOF, the Delassus
  // estimate w = 1/m is exact and therefore v = v*⋅σ/(1 + σ).
  double ExpectedStictionVelocityFromRest(double applied_force, double inertia,
                                          double sigma) const {
    const double v_free = kTimeStep * applied_force / inertia;
    return v_free * sigma / (1.0 + sigma);
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
  const RevoluteJoint<double>* wheel_{nullptr};
  const PrismaticJoint<double>* slider_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};
  const JointActuator<double>* wheel_actuator_{nullptr};
  const JointActuator<double>* slider_actuator_{nullptr};
};

// Verifies the driver adds one SapJointFrictionConstraint per single DOF joint
// with non-zero dry friction, using the value stored in the context and the
// right indexing within each tree.
TEST_F(SapDriverJointFrictionTest, ConstraintsAreAdded) {
  const double wheel_friction = 2.0 * kWheelFriction;
  wheel_->SetDryFriction(context_.get(), wheel_friction);

  // Trees are numbered in the order their base bodies were added, and the
  // constraints are added in joint index order.
  struct ExpectedConstraint {
    int tree;
    int tree_nv;
    int tree_dof;
    double friction;
  };
  const std::vector<ExpectedConstraint> expected{
      {0, 1, 0, wheel_friction},   // Wheel.
      {1, 1, 0, kSliderFriction},  // Slider.
      {2, 2, 1, kElbowFriction},   // Elbow, the second DOF of the arm.
  };
  const std::vector<const SapJointFrictionConstraint<double>*> constraints =
      EvalConstraints();
  ASSERT_EQ(std::ssize(constraints), std::ssize(expected));
  for (int i = 0; i < std::ssize(expected); ++i) {
    EXPECT_EQ(constraints[i]->num_cliques(), 1);
    EXPECT_EQ(constraints[i]->first_clique(), expected[i].tree);
    EXPECT_EQ(constraints[i]->num_velocities(0), expected[i].tree_nv);
    EXPECT_EQ(constraints[i]->clique_dof(), expected[i].tree_dof);
    EXPECT_EQ(constraints[i]->parameters().tau_c, expected[i].friction);
  }
}

// Locked joints do not lead to constraints, both when the whole tree is locked
// (the slider) and when other DOFs of the tree are not locked (the elbow).
TEST_F(SapDriverJointFrictionTest, LockedJointsAreSkipped) {
  slider_->Lock(context_.get());
  elbow_->Lock(context_.get());
  const std::vector<const SapJointFrictionConstraint<double>*> constraints =
      EvalConstraints();
  ASSERT_EQ(constraints.size(), 1);
  EXPECT_EQ(constraints[0]->parameters().tau_c, kWheelFriction);
}

// Under an applied force below the friction bound, joints stay in stiction:
// friction balances the applied force, and the joints only creep at the small
// velocity set by the regularization of friction.
TEST_F(SapDriverJointFrictionTest, Stiction) {
  const double wheel_torque = 0.5 * kWheelFriction;
  const double slider_force = -0.8 * kSliderFriction;
  SetActuation(wheel_torque, slider_force);

  // The constraints are added in joint index order: wheel, slider and elbow.
  const std::vector<const SapJointFrictionConstraint<double>*> constraints =
      EvalConstraints();
  ASSERT_EQ(constraints.size(), 3);
  const double wheel_sigma = constraints[0]->parameters().sigma;
  const double slider_sigma = constraints[1]->parameters().sigma;

  const ContactSolverResults<double>& results =
      manager_->EvalContactSolverResults(*context_);
  const double wheel_v = results.v_next(wheel_->velocity_start());
  const double slider_v = results.v_next(slider_->velocity_start());
  const double wheel_v_expected = ExpectedStictionVelocityFromRest(
      wheel_torque, kWheelInertia, wheel_sigma);
  const double slider_v_expected =
      ExpectedStictionVelocityFromRest(slider_force, kSliderMass, slider_sigma);
  EXPECT_NEAR(wheel_v, wheel_v_expected,
              kTolerance * std::abs(wheel_v_expected));
  EXPECT_NEAR(slider_v, slider_v_expected,
              kTolerance * std::abs(slider_v_expected));
}

// Under an applied force above the friction bound, joints slide with friction
// saturated at its bound and opposing motion.
TEST_F(SapDriverJointFrictionTest, Sliding) {
  const double wheel_torque = 3.0 * kWheelFriction;
  const double slider_force = -2.5 * kSliderFriction;
  SetActuation(wheel_torque, slider_force);

  const ContactSolverResults<double>& results =
      manager_->EvalContactSolverResults(*context_);
  const double wheel_v = results.v_next(wheel_->velocity_start());
  const double slider_v = results.v_next(slider_->velocity_start());
  const double wheel_v_expected = ExpectedSlidingVelocityFromRest(
      wheel_torque, kWheelInertia, kWheelFriction);
  const double slider_v_expected = ExpectedSlidingVelocityFromRest(
      slider_force, kSliderMass, kSliderFriction);
  EXPECT_NEAR(wheel_v, wheel_v_expected,
              kTolerance * std::abs(wheel_v_expected));
  EXPECT_NEAR(slider_v, slider_v_expected,
              kTolerance * std::abs(slider_v_expected));
}

// The scalar converted plant on AutoDiffXd, with its own default dry friction,
// produces the same velocities as the double plant.
TEST_F(SapDriverJointFrictionTest, AutoDiffXd) {
  const double wheel_torque = 3.0 * kWheelFriction;   // Sliding.
  const double slider_force = 0.5 * kSliderFriction;  // Stiction.
  SetActuation(wheel_torque, slider_force);

  const VectorXd v_ad = StepAutoDiffXd();
  const VectorXd& v = manager_->EvalContactSolverResults(*context_).v_next;
  EXPECT_TRUE(CompareMatrices(v_ad, v, kTolerance * v.norm()));
  // Sanity check that the model did move.
  EXPECT_GT(v(wheel_->velocity_start()), 0.0);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
