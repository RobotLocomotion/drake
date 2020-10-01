#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {

using multibody::Parser;
using systems::Context;
using systems::BasicVector;

namespace multibody {
namespace {

// Fixture to set up two Kuka iiwa arm instances, each with a Schunk wsg
// gripper and welded to the world at the base link. The model contained in
// `plant_` contains the default model from parsing. The model contained in
// `plant_ri_` contains the identical model, but with reflected inertia values
// added to each joint actuator.
class MultibodyPlantReflectedInertiaTests : public ::testing::Test {
 public:
  // @param[in] reflected_inertia Reflected inertia values to be added to the
  // joint actuators of the model. It is expected that reflected_inertia.size()
  // == plant_ri_.num_actuators() and that the index in the vector corresponds
  // with the joint actuator index.
  void LoadBothModelsSetStateAndFinalize(
      const VectorX<double>& reflected_inertia) {
    LoadIiwaWithGripper(&plant_);
    LoadIiwaWithGripper(&plant_ri_);
    AddInReflectedInertia(&plant_ri_, reflected_inertia);

    plant_.Finalize();
    plant_ri_.Finalize();
    context_ = plant_.CreateDefaultContext();
    context_ri_ = plant_ri_.CreateDefaultContext();
    SetArbitraryState(plant_, context_.get());
    SetArbitraryState(plant_ri_, context_ri_.get());
  }

 private:
  void LoadIiwaWithGripper(MultibodyPlant<double>* plant) {
    const char kArmSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf";

    const char kWsg50SdfPath[] =
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

    Parser parser(plant);
    arm_model = parser.AddModelFromFile(FindResourceOrThrow(kArmSdfPath));

    // Add the gripper.
    gripper_model = parser.AddModelFromFile(FindResourceOrThrow(kWsg50SdfPath));

    const auto& base_body = plant->GetBodyByName("iiwa_link_0", arm_model);
    const auto& end_effector = plant->GetBodyByName("iiwa_link_7", arm_model);
    const auto& gripper_body = plant->GetBodyByName("body", gripper_model);
    plant->WeldFrames(plant->world_frame(), base_body.body_frame());
    plant->WeldFrames(end_effector.body_frame(), gripper_body.body_frame());
  }

  void AddInReflectedInertia(MultibodyPlant<double>* plant,
                             const VectorX<double>& reflected_inertia) {
    for (JointActuatorIndex index(0); index < plant->num_actuators(); ++index) {
      JointActuator<double>& joint_actuator =
          plant->get_mutable_joint_actuator(index);
      joint_actuator.set_reflected_inertia(reflected_inertia(index));
    }
  }

  void SetArbitraryState(const MultibodyPlant<double>& plant,
                         Context<double>* context) {
    for (JointIndex joint_index(0); joint_index < plant.num_joints();
         ++joint_index) {
      const Joint<double>& joint = plant.get_joint(joint_index);
      // This model only has weld, prismatic, and revolute joints.
      if (joint.type_name() == "revolute") {
        const RevoluteJoint<double>& revolute_joint =
            dynamic_cast<const RevoluteJoint<double>&>(joint);
        // Arbitrary angle and angular rate.
        revolute_joint.set_angle(context, 0.5 * joint_index);
        revolute_joint.set_angular_rate(context, 0.5 * joint_index);
      } else if (joint.type_name() == "prismatic") {
        const PrismaticJoint<double>& prismatic_joint =
            dynamic_cast<const PrismaticJoint<double>&>(joint);
        // Arbitrary joint translation and translation rate.
        prismatic_joint.set_translation(context, 0.5 * joint_index);
        prismatic_joint.set_translation_rate(context, 0.5 * joint_index);
      }
    }
  }

 protected:
  const int kNumJoints = 9;

  MultibodyPlant<double> plant_{0.0};
  MultibodyPlant<double> plant_ri_{0.0};
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<Context<double>> context_ri_;

  ModelInstanceIndex arm_model;
  ModelInstanceIndex gripper_model;
};

// This test is used to verify the expected difference in the mass matrix
// (computed by `CalcMassMatrix`) between the model with reflected inertias and
// the model without.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcMassMatrix) {
  // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Map reflected inertias from actuation indices to general velocity indices.
  MatrixX<double> B = plant_.MakeActuationMatrix();
  VectorX<double> reflected_inertia_v = B*reflected_inertia;

  // Compute mass matrix for each model.
  MatrixX<double> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(*context_, &M);

  MatrixX<double> M_ri(plant_ri_.num_velocities(), plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrix(*context_ri_, &M_ri);

  // Compute a suitable tolerance scaled with the norm of the mass matrix.
  // Since .norm() computes the Frobenius norm, and num_velocities() is the
  // squared root of the number of elements in the matrix, this tolerance is
  // effectively being scaled by the RMS value of the elements in the mass
  // matrix.
  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon() *
                            M.norm() / plant_.num_velocities();

  // The difference between the mass matrix of the system with reflected inertia
  // and the system without is a diagonal matrix whose diagonal elements are the
  // reflected inertias.
  MatrixX<double> Mdiff = M_ri - M;
  MatrixX<double> Mdiagonal = reflected_inertia_v.asDiagonal();
  EXPECT_TRUE(CompareMatrices(Mdiff, Mdiagonal, kTolerance,
                              MatrixCompareType::relative));
}

// This test is used to verify the expected difference in the mass matrix
// (computed by `CalcMassMatrixViaInverseDynamics`) between the model with
// reflected inertias and the model without.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcMassMatrixViaInverseDynamics) {
    // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Map reflected inertias from actuation indices to general velocity indices.
  MatrixX<double> B = plant_.MakeActuationMatrix();
  VectorX<double> reflected_inertia_v = B*reflected_inertia;

  // Compute mass matrix for each model.
  MatrixX<double> M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrixViaInverseDynamics(*context_, &M);

  MatrixX<double> M_ri(plant_ri_.num_velocities(), plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrixViaInverseDynamics(*context_ri_, &M_ri);

  // Compute a suitable tolerance scaled with the norm of the mass matrix.
  // Since .norm() computes the Frobenius norm, and num_velocities() is the
  // squared root of the number of elements in the matrix, this tolerance is
  // effectively being scaled by the RMS value of the elements in the mass
  // matrix.
  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon() *
                            M.norm() / plant_.num_velocities();

  // The difference between the mass matrix of the system with reflected inertia
  // and the system without is a diagonal matrix whose diagonal elements are the
  // reflected inertias.
  MatrixX<double> Mdiff = M_ri - M;
  MatrixX<double> Mdiagonal = reflected_inertia_v.asDiagonal();
  EXPECT_TRUE(CompareMatrices(Mdiff, Mdiagonal, kTolerance,
                              MatrixCompareType::relative));
}

// This test verifies the expected difference in output from inverse dynamics
// between the two models.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcInverseDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Map reflected inertias from actuation indices to general velocity indices.
  MatrixX<double> B = plant_.MakeActuationMatrix();
  VectorX<double> reflected_inertia_v = B*reflected_inertia;

  // Given that the difference between the two models are the diagonal elements
  // of the mass matrix (M for the original model and M̃ for the model with
  // reflected inertia), the difference in tau output by inverse dynamics is
  // precisely (M̃ - M)v̇. Setting v̇ to 𝟏 will extract the diagonal elements
  // of (M̃ - M), which are exactly the values in `reflected_inertia`.
  VectorX<double> vdot(plant_.num_velocities());
  vdot.setOnes();

  // Pass all external forces to inverse dynamics. The only external forces come
  // from force elements because there is no actuation nor contact forces.
  MultibodyForces<double> forces(plant_);
  MultibodyForces<double> forces_ri(plant_ri_);
  plant_.CalcForceElementsContribution(*context_, &forces);
  plant_ri_.CalcForceElementsContribution(*context_ri_, &forces_ri);

  VectorX<double> tau = plant_.CalcInverseDynamics(*context_, vdot, forces);
  VectorX<double> tau_ri =
      plant_ri_.CalcInverseDynamics(*context_ri_, vdot, forces_ri);

  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon();

  VectorX<double> tau_diff = tau_ri - tau;
  EXPECT_TRUE(CompareMatrices(tau_diff, reflected_inertia_v, kTolerance,
                              MatrixCompareType::relative));
}

// This test verifies that forward and inverse dynamics are consistent by
// propagating the output of forward dynamics through inverse dynamics and
// checking that the resulting generalized forces match the input.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcForwardAndInverseDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Map reflected inertias from actuation indices to general velocity indices.
  MatrixX<double> B = plant_.MakeActuationMatrix();

  // Set actuation forces to random values.
  const VectorX<double> tau_arm =
      VectorX<double>::Random(plant_ri_.num_actuated_dofs(arm_model));
  const VectorX<double> tau_gripper =
      VectorX<double>::Random(plant_ri_.num_actuated_dofs(gripper_model));
  plant_ri_.get_actuation_input_port(arm_model).FixValue(context_ri_.get(),
                                                         tau_arm);
  plant_ri_.get_actuation_input_port(gripper_model)
      .FixValue(context_ri_.get(), tau_gripper);

  // Map tau_expected from actuation indices to general velocity indices.
  VectorX<double> tau_expected(tau_arm.size() + tau_gripper.size());
  tau_expected << tau_arm, tau_gripper;
  tau_expected = B*tau_expected;

  // Calculate forward dynamics with the Articulated Body Algorithm.
  VectorX<double> vdot_aba =
      plant_ri_.get_generalized_acceleration_output_port()
          .Eval<systems::BasicVector<double>>(*context_ri_)
          .CopyToVector();

  // Compute tau = M(q)v̇ + C(q, v)v - tau_app via inverse dynamics. By setting
  // tau_app to just force element contributions, the lhs (tau) is exactly the
  // actuation input forces.
  MultibodyForces<double> forces(plant_);
  plant_ri_.CalcForceElementsContribution(*context_ri_, &forces);
  const VectorX<double> tau_id =
      plant_ri_.CalcInverseDynamics(*context_ri_, vdot_aba, forces);

  // Compare tau_id and tau_expected
  const double kTolerance = 100.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(tau_id, tau_expected, kTolerance,
                              MatrixCompareType::relative));
}

// This test validates the output of forward dynamics (the articulated body
// algorithm) by explicitly solving M(q)v̇ = tau
// (where tau = -C(q, v)v + tau_app).
TEST_F(MultibodyPlantReflectedInertiaTests, CalcForwardDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Set actuation forces to 0.
  const VectorX<double> tau_arm =
      VectorX<double>::Zero(plant_ri_.num_actuated_dofs(arm_model));
  const VectorX<double> tau_gripper =
      VectorX<double>::Zero(plant_ri_.num_actuated_dofs(gripper_model));
  plant_ri_.get_actuation_input_port(arm_model).FixValue(context_ri_.get(),
                                                         tau_arm);
  plant_ri_.get_actuation_input_port(gripper_model)
      .FixValue(context_ri_.get(), tau_gripper);

  // Calculate forward dynamics with the Articulated Body Algorithm.
  VectorX<double> vdot_aba =
      plant_ri_.get_generalized_acceleration_output_port()
          .Eval<systems::BasicVector<double>>(*context_ri_)
          .CopyToVector();

  // Calculate forward dynamics by explicitlty solving M(q)v̇ = tau_id.
  // Construct M, the mass matrix.
  const int nv = plant_ri_.num_velocities();
  MatrixX<double> M(nv, nv);
  plant_ri_.CalcMassMatrixViaInverseDynamics(*context_ri_, &M);

  // Compute force element contributions.
  MultibodyForces<double> forces(plant_);
  plant_ri_.CalcForceElementsContribution(*context_ri_, &forces);

  // Compute tau = C(q, v)v - tau_app via inverse dynamics.
  const VectorX<double> zero_vdot = VectorX<double>::Zero(nv);
  const VectorX<double> tau_id =
      plant_ri_.CalcInverseDynamics(*context_ri_, zero_vdot, forces);

  // Solve for vdot explicitly.
  VectorX<double> vdot = M.llt().solve(-tau_id);

  // We estimate the difference between vdot and vdot_aba to be in the
  // order of machine epsilon times the condition number "kappa" of the mass
  // matrix.
  const double kTolerance =
      (1.0 / M.llt().rcond()) * std::numeric_limits<double>::epsilon();

  EXPECT_TRUE(
      CompareMatrices(vdot_aba, vdot, kTolerance, MatrixCompareType::relative));
}

// This test verfies that reflected inertia values are propagated through system
// scalar conversion.
TEST_F(MultibodyPlantReflectedInertiaTests, ScalarConversion) {
  // Arbitrary reflected inertia values.
  VectorX<double> reflected_inertia(kNumJoints);
  reflected_inertia << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(reflected_inertia);

  // Compute mass matrix of the original system.
  MatrixX<double> M_ri(plant_ri_.num_velocities(), plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrix(*context_ri_, &M_ri);

  // Scalar-convert the model and copy the context of the original system.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant_ri_);
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*context_ri_);

  // Compute the mass matrix of the scalar converted system.
  MatrixX<AutoDiffXd> M_autodiff(plant_autodiff->num_velocities(),
                                 plant_autodiff->num_velocities());
  plant_autodiff->CalcMassMatrix(*context_autodiff, &M_autodiff);

  // Verify that the mass matrices are equal.
  const double kTolerance = 10.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M_ri, M_autodiff, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
