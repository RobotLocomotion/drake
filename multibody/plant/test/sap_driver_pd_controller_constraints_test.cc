#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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

namespace contact_solvers {
namespace internal {
// N.B. What namespace this operator is in is important for ADL.
bool operator==(const Configuration& a, const Configuration& b) {
  if (a.clique != b.clique) return false;
  if (a.clique_dof != b.clique_dof) return false;
  if (a.clique_nv != b.clique_nv) return false;
  if (a.q0 != b.q0) return false;
  if (a.qd != b.qd) return false;
  if (a.vd != b.vd) return false;
  if (a.u0 != b.u0) return false;
  return true;
}
}  // namespace internal
}  // namespace contact_solvers

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

namespace kcov339_avoidance_magic {

// Test fixture that sets up a model of an IIWA arm with PD controlled gripper.
// Its purpose is to verify that the SAP driver defines constraints
// appropriately to model the PD controllers on the gripper.
class ActuatedIiiwaArmTest : public ::testing::Test {
 public:
  void SetUp() override {
    const char kArmSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf";

    const char kWsg50SdfPath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    // Make a discrete model.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.01 /* update period */);
    // Use the SAP solver. Thus far only SAP support the modeling of PD
    // controllers.
    plant_->set_discrete_contact_approximation(
        DiscreteContactApproximation::kSap);

    Parser parser(plant_.get());

    // Add the arm.
    arm_model_ = parser.AddModels(FindResourceOrThrow(kArmSdfPath)).at(0);

    // Add the gripper.
    gripper_model_ = parser.AddModels(FindResourceOrThrow(kWsg50SdfPath)).at(0);

    const auto& base_body = plant_->GetBodyByName("iiwa_link_0", arm_model_);
    const auto& end_effector = plant_->GetBodyByName("iiwa_link_7", arm_model_);
    const auto& gripper_body = plant_->GetBodyByName("body", gripper_model_);
    plant_->WeldFrames(plant_->world_frame(), base_body.body_frame());
    plant_->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());

    // Set PD controllers for the gripper.
    SetGripperModel();

    plant_->Finalize();

    // We set the manager programmatically in order to have access to its
    // instance.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    context_ = plant_->CreateDefaultContext();
  }

  void SetGripperModel() {
    for (JointActuatorIndex actuator_index(0);
         actuator_index < plant_->num_actuators(); ++actuator_index) {
      JointActuator<double>& actuator =
          plant_->get_mutable_joint_actuator(actuator_index);
      if (actuator.model_instance() == gripper_model_) {
        actuator.set_controller_gains({kProportionalGain_, kDerivativeGain_});
      }
    }
  }

  const SapDriver<double>& sap_driver() const {
    return CompliantContactManagerTester::sap_driver(*manager_);
  }

 protected:
  const int kKukaNumPositions_{7};
  const int kGripperNumPositions_{2};
  const double kProportionalGain_{10000.0};
  const double kDerivativeGain_{100.0};
  const double kEffortLimit_{80.0};  // Defined in schunk_wsg_50.sdf
  std::unique_ptr<MultibodyPlant<double>> plant_;
  ModelInstanceIndex arm_model_;
  ModelInstanceIndex gripper_model_;
  CompliantContactManager<double>* manager_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// We verify that the SAP driver properly defined constraints to model the PD
// controllers in the gripper fingers.
TEST_F(ActuatedIiiwaArmTest, VerifyConstraints) {
  // We expect each of the 1-DOF joints to be actuated.
  EXPECT_EQ(plant_->num_actuators(), plant_->num_velocities());

  // Sanity check we only defined PD controllers for the grippers DOFs.
  int num_controlled_actuators = 0;
  for (JointActuatorIndex a(0); a < plant_->num_actuators(); ++a) {
    const JointActuator<double>& actuator = plant_->get_joint_actuator(a);
    if (actuator.has_controller()) ++num_controlled_actuators;
  }
  EXPECT_EQ(num_controlled_actuators, 2);

  // The actuation input port for the arm is required to be connected.
  const VectorXd arm_u =
      VectorXd::LinSpaced(kKukaNumPositions_, 1.0, kKukaNumPositions_);
  plant_->get_actuation_input_port(arm_model_).FixValue(context_.get(), arm_u);

  // Since the gripper has controllers, the desired state input port must be
  // connected.
  const VectorXd gripper_xd = (VectorXd(4) << 1., 2., 3, 4.).finished();
  plant_->get_desired_state_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_xd);

  // Arbitrary feed-forward term for testing.
  const VectorXd gripper_ff = (VectorXd(2) << 5., 6.).finished();
  plant_->get_actuation_input_port(gripper_model_)
      .FixValue(context_.get(), gripper_ff);

  const ContactProblemCache<double>& problem_cache =
      SapDriverTest::EvalContactProblemCache(sap_driver(), *context_);
  const SapContactProblem<double>& problem = *problem_cache.sap_problem;

  // We should at least have two constraints to model the controllers.
  // Additional constraints will correspond to joint limits.
  EXPECT_GE(problem.num_constraints(), 2);
  EXPECT_GE(problem.num_constraint_equations(), 2);

  auto make_finger_config = [&](const std::string& name) {
    const int dof = plant_->GetJointByName(name).velocity_start();
    const int gripper_dof = dof - kKukaNumPositions_;
    const int clique = 0;  // Only one kinematic tree in this model.
    const int clique_dof = dof;
    const int clique_nv = 9;  // 7 Kuka DOFs + 2 gripper DOFs.
    const VectorXd q = plant_->GetPositions(*context_);
    const double q0 = q[dof];  // For this model nq = nv.
    const double qd = gripper_xd(gripper_dof);
    const double vd = gripper_xd(2 + gripper_dof);
    const double u_ff = gripper_ff(gripper_dof);
    SapPdControllerConstraint<double>::Configuration config{
        clique, clique_dof, clique_nv, q0, qd, vd, u_ff};
    return config;
  };

  const SapPdControllerConstraint<double>::Configuration left_finger_config =
      make_finger_config("left_finger_sliding_joint");
  const SapPdControllerConstraint<double>::Configuration right_finger_config =
      make_finger_config("right_finger_sliding_joint");

  for (int k = 0; k < problem.num_constraints(); ++k) {
    const auto* constraint =
        dynamic_cast<const SapPdControllerConstraint<double>*>(
            &problem.get_constraint(k));
    if (constraint != nullptr) {
      const auto& p = constraint->parameters();
      EXPECT_EQ(p.Kp(), kProportionalGain_);
      EXPECT_EQ(p.Kd(), kDerivativeGain_);
      EXPECT_EQ(p.effort_limit(), kEffortLimit_);

      // Always one clique for PD controllers.
      EXPECT_EQ(constraint->num_cliques(), 1);
      // There is only one kinematic tree in this model, thus the driver will
      // only define a single clique.
      EXPECT_EQ(constraint->first_clique(), 0);
      EXPECT_THROW(constraint->second_clique(), std::exception);

      const auto& c = constraint->configuration();
      EXPECT_EQ(c.clique, 0);  // There is only one clique.
      // Either of the two gripper DOFs.
      EXPECT_TRUE(c.clique_dof == 7 || c.clique_dof == 8);
      EXPECT_EQ(c.clique_nv, 9);  // 7 Kuka DOFs + 2 gripper DOFs.

      if (c.clique_dof == left_finger_config.clique_dof) {
        EXPECT_EQ(c, left_finger_config);
      } else {
        EXPECT_EQ(c, right_finger_config);
      }
    }
  }
}

}  // namespace kcov339_avoidance_magic
}  // namespace internal
}  // namespace multibody
}  // namespace drake
