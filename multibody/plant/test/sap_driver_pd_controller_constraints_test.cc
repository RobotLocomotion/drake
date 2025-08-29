#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_pd_controller_constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"

/* @file This file tests SapDriver's support for PD controller constraints.

  Constraints are only supported by the SAP solver. Therefore, to exercise the
  relevant code paths, we arbitrarily choose one contact approximation that uses
  the SAP solver. More precisely, in the unit tests below we call
  set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
  MultibodyPlant used for testing, before constraints are added. */

using drake::math::RigidTransformd;
using drake::multibody::Parser;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapPdControllerConstraint;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Configuration = SapPdControllerConstraint<double>::Configuration;

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

// Test fixture that sets up a model of an IIWA arm with PD controlled gripper.
// Its purpose is to verify that the SAP driver defines constraints
// appropriately to model the PD controllers on the gripper.
class ActuatedIiwaArmTest : public ::testing::Test {
 public:
  void SetUp() override {
    const char kArmSdfUrl[] =
        "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
    const char kWsg50SdfUrl[] =
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    // Make a discrete model.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.01 /* update period */);
    // Use the SAP solver. Thus far only SAP support the modeling of PD
    // controllers.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    Parser parser(plant_.get());

    // Add the arm.
    arm_model_ = parser.AddModelsFromUrl(kArmSdfUrl).at(0);

    // Add the gripper.
    gripper_model_ = parser.AddModelsFromUrl(kWsg50SdfUrl).at(0);

    const auto& base_body = plant_->GetBodyByName("iiwa_link_0", arm_model_);
    const auto& end_effector = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
    plant_->WeldFrames(plant_->world_frame(), base_body.body_frame());
    plant_->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    SetPdGainsForGripperModel();
    SetPdGainsForArmModel();

    plant_->Finalize();

    // We set the manager programmatically in order to have access to its
    // instance.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    context_ = plant_->CreateDefaultContext();
  }

  // Sets both actuators to have PD control.
  void SetPdGainsForGripperModel() {
    for (JointActuatorIndex actuator_index :
         plant_->GetJointActuatorIndices()) {
      JointActuator<double>& actuator =
          plant_->get_mutable_joint_actuator(actuator_index);
      if (actuator.model_instance() == gripper_model_) {
        actuator.set_controller_gains(
            {kGripperProportionalGain_, kGripperDerivativeGain_});
      }
    }
  }

  // Sets only joints 2 and 4 of the arm to have PD control, even though all
  // joints have (feed-forward) actuation.
  void SetPdGainsForArmModel() {
    for (JointActuatorIndex actuator_index :
         plant_->GetJointActuatorIndices()) {
      JointActuator<double>& actuator =
          plant_->get_mutable_joint_actuator(actuator_index);
      if (actuator.joint().name() == "iiwa_joint_2" ||
          actuator.joint().name() == "iiwa_joint_4") {
        actuator.set_controller_gains(
            {kArmProportionalGain_, kArmDerivativeGain_});
      }
    }
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

 protected:
  const int kKukaNumPositions_{7};
  const int kGripperNumPositions_{2};
  const double kGripperProportionalGain_{10000.0};
  const double kGripperDerivativeGain_{100.0};
  const double kGripperEffortLimit_{80.0};  // Defined in schunk_wsg_50.sdf
  const double kArmProportionalGain_{15000.0};
  const double kArmDerivativeGain_{300.0};
  // Efforts for second and fourth joints defined in iiwa14_no_collision.sdf
  const double kArmEffortLimit2_{320.0};
  const double kArmEffortLimit4_{176.0};
  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex gripper_model_;
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// We verify that the SAP driver properly defined constraints to model the PD
// controllers in the gripper fingers. We build a model in which only a subset
// of non-consecutive joints in the arm have PD control. We do this to stress
// test the proper assembly of the resulting SAP problem.
TEST_F(ActuatedIiwaArmTest, VerifyConstraints) {
  // We expect each of the 1-DOF joints to be actuated.
  EXPECT_EQ(plant_->num_actuators(), plant_->num_velocities());

  // Sanity check the number of actuators.
  EXPECT_EQ(plant_->num_actuators(gripper_model_), 2);
  EXPECT_EQ(plant_->num_actuators(arm_model_), 7);

  // We must provide desired state values for all actuators, whether they have
  // PD control or not.
  // Values for non-controlled actuators are ignored. We fill those in with NaNs
  // and verify they do not propagate below.
  VectorXd plant_qd = VectorXd::LinSpaced(9, 1.0, 9.0);
  VectorXd plant_vd = VectorXd::LinSpaced(9, 10.0, 18.0);
  // The first actuator in the arm is not PD-controlled.
  plant_qd[0] = NAN;
  plant_vd[0] = NAN;

  // Plant feed-forward actuation, arm (7) + gripper (2)
  const VectorXd plant_uff = VectorXd::LinSpaced(9, 7.0, 13.0);

  // Desired state for the gripper.
  const VectorXd gripper_xd = (VectorXd(2 * kGripperNumPositions_)
                                   << plant_qd.tail(kGripperNumPositions_),
                               plant_vd.tail(kGripperNumPositions_))
                                  .finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // Feed-forward actuation for the gripper.
  const VectorXd gripper_uff = plant_uff.tail(kGripperNumPositions_);
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_uff);

  // Desired state for the arm.
  const VectorXd arm_xd =
      (VectorXd(2 * kKukaNumPositions_) << plant_qd.head(kKukaNumPositions_),
       plant_vd.head(kKukaNumPositions_))
          .finished();
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Feed-forward actuation for the arm.
  const VectorXd arm_uff = plant_uff.head(kKukaNumPositions_);
  plant_->get_actuation_input_port(arm_model_)
      .FixValue(context_.get(), arm_uff);

  // Evaluate the contact problem so that we can verify it has the expected
  // constraints.
  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // We should at least have four constraints to model the PD controllers.
  // Additional constraints will correspond to joint limits.
  EXPECT_GE(problem.num_constraints(), 4);
  EXPECT_GE(problem.num_constraint_equations(), 4);

  auto make_config = [&](int dof) {
    const int clique = 0;  // Only one kinematic tree in this model.
    const int clique_dof = dof;
    const int clique_nv = kKukaNumPositions_ + kGripperNumPositions_;
    const VectorXd q = plant_->GetPositions(*context_);
    const double q0 = q[dof];  // For this model nq = nv.
    const double qd = plant_qd(dof);
    const double vd = plant_vd(dof);
    const double u_ff = plant_uff(dof);
    SapPdControllerConstraint<double>::Configuration config{
        clique, clique_dof, clique_nv, q0, qd, vd, u_ff};
    return config;
  };

  for (int k = 0; k < problem.num_constraints(); ++k) {
    const auto* constraint =
        dynamic_cast<const SapPdControllerConstraint<double>*>(
            &problem.get_constraint(k));
    if (constraint != nullptr) {
      const auto& c = constraint->configuration();
      // Verify that NaN desired states for non-controlled actuators do not
      // propagate into any constraint.
      using std::isnan;
      EXPECT_FALSE(isnan(c.qd));
      EXPECT_FALSE(isnan(c.vd));

      // Verify expected values.
      const SapPdControllerConstraint<double>::Configuration expected_config =
          make_config(c.clique_dof);
      EXPECT_EQ(c, expected_config);

      // Always one clique for PD controllers.
      EXPECT_EQ(constraint->num_cliques(), 1);
      // There is only one kinematic tree in this model, thus the driver will
      // only define a single clique.
      EXPECT_EQ(constraint->first_clique(), 0);
      EXPECT_THROW(constraint->second_clique(), std::exception);

      const auto& p = constraint->parameters();
      if (c.clique_dof >= 7) {  // Gripper
        EXPECT_EQ(p.Kp(), kGripperProportionalGain_);
        EXPECT_EQ(p.Kd(), kGripperDerivativeGain_);
        EXPECT_EQ(p.effort_limit(), kGripperEffortLimit_);
      } else {  // Arm
        EXPECT_EQ(p.Kp(), kArmProportionalGain_);
        EXPECT_EQ(p.Kd(), kArmDerivativeGain_);
        const double expected_limit =
            c.clique_dof == 1 ? kArmEffortLimit2_ : kArmEffortLimit4_;
        EXPECT_EQ(p.effort_limit(), expected_limit);
      }
    }
  }
}

TEST_F(ActuatedIiwaArmTest, ZeroPTerm) {
  // Add a velocity-only PD controller.
  const JointActuator<double>& actuator =
      plant_->GetJointActuatorByName("iiwa_joint_3");
  plant_->get_mutable_joint_actuator(actuator.index())
      .set_controller_gains({.p = 0, .d = kArmDerivativeGain_});

  // Set desired state for the arm.
  const VectorXd arm_xd = VectorXd::Zero(2 * kKukaNumPositions_);
  plant_->get_desired_state_input_port(arm_model_)
      .FixValue(context_.get(), arm_xd);

  // Check that the constraint is allowed. All we need is a non-crash test, to
  // guard against input sanitization bugs.
  EXPECT_NO_THROW(
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
