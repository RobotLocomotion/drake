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
using systems::BasicVector;
using systems::Context;

namespace multibody {
namespace {

// Fixture to set up two Kuka iiwa arm instances, each with a Schunk wsg
// gripper and welded to the world at the base link. The model contained in
// `plant_` contains the default model from parsing. The model contained in
// `plant_ri_` contains the identical model, but with reflected inertia values
// added to each joint actuator.
class MultibodyPlantReflectedInertiaTests : public ::testing::Test {
 public:
  // @param[in] rotor_inertias Individual rotor inertia values for each joint
  //   actuator.
  // @param[in] gear_ratios Individual gear ratio values for each joint
  //   actuator.
  // @pre It is expected that:
  //   rotor_inertias.size() == gear_ratios.size() == plant_ri_.num_actuators()
  //   and that the index in the vector corresponds with the joint actuator
  //   index.
  //
  // See the section 'Reflected Inertia' in documentation for
  // drake::multibody::JointActuator for more details.
  void LoadBothModelsSetStateAndFinalize(const VectorX<double>& rotor_inertias,
                                         const VectorX<double>& gear_ratios) {
    LoadIiwaWithGripper(&plant_);
    LoadIiwaWithGripper(&plant_ri_);
    AddInReflectedInertia(&plant_ri_, rotor_inertias, gear_ratios);

    plant_.Finalize();
    plant_ri_.Finalize();
    context_ = plant_.CreateDefaultContext();
    context_ri_ = plant_ri_.CreateDefaultContext();
    // Set the plant_'s context and the plant_ri_'s context to the same
    // arbitrary state.
    SetArbitraryState(plant_, context_.get());
    SetArbitraryState(plant_ri_, context_ri_.get());
  }

 private:
  void LoadIiwaWithGripper(MultibodyPlant<double>* plant) {
    DRAKE_DEMAND(plant != nullptr);
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
                             const VectorX<double>& rotor_inertias,
                             const VectorX<double>& gear_ratios) {
    DRAKE_DEMAND(plant != nullptr);
    for (JointActuatorIndex index(0); index < plant->num_actuators(); ++index) {
      JointActuator<double>& joint_actuator =
          plant->get_mutable_joint_actuator(index);
      joint_actuator.set_default_rotor_inertia(rotor_inertias(int{index}));
      joint_actuator.set_default_gear_ratio(gear_ratios(int{index}));
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

VectorX<double> MapVectorFromActuatorIndicesToGeneralizedVelocityIndices(
    const MultibodyPlant<double>& plant, const VectorX<double>& v) {
  const MatrixX<double> B = plant.MakeActuationMatrix();
  return B * v;
}

// This test verifies the expected difference in the mass matrix computed by
// MultibodyPlant::CalcMassMatrix() between the models with and without
// reflected inertia.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcMassMatrix) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Map reflected inertias from actuation indices to generalized velocity
  // indices.
  const VectorX<double> reflected_inertia_with_generalized_velocity_indices =
      MapVectorFromActuatorIndicesToGeneralizedVelocityIndices(
          plant_, reflected_inertia_with_actuator_indices);

  // Compute mass matrix for each model.
  MatrixX<double> mass_matrix(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(*context_, &mass_matrix);

  MatrixX<double> mass_matrix_ri(plant_ri_.num_velocities(),
                                 plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrix(*context_ri_, &mass_matrix_ri);

  // Compute a suitable tolerance scaled with the norm of the mass matrix.
  // Since .norm() computes the Frobenius norm, and num_velocities() is the
  // squared root of the number of elements in the matrix, this tolerance is
  // effectively being scaled by the RMS value of the elements in the mass
  // matrix.
  const double kTolerance = 16.0 * std::numeric_limits<double>::epsilon() *
                            mass_matrix.norm() / plant_.num_velocities();

  // The difference between the mass matrix of the system with reflected inertia
  // and the system without is a diagonal matrix whose diagonal elements are the
  // reflected inertias.
  const MatrixX<double> Mdiff = mass_matrix_ri - mass_matrix;
  const MatrixX<double> Mdiagonal =
      reflected_inertia_with_generalized_velocity_indices.asDiagonal();
  EXPECT_TRUE(CompareMatrices(Mdiff, Mdiagonal, kTolerance,
                              MatrixCompareType::relative));
}

// This test verifies the expected difference in the mass matrix computed by
// MultibodyPlant::CalcMassMatrixViaInverseDynamics() between the models
// with and without reflected inertia.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcMassMatrixViaInverseDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Map reflected inertias from actuation indices to generalized velocity
  // indices.
  const VectorX<double> reflected_inertia_with_generalized_velocity_indices =
      MapVectorFromActuatorIndicesToGeneralizedVelocityIndices(
          plant_, reflected_inertia_with_actuator_indices);

  // Compute mass matrix for each model.
  MatrixX<double> mass_matrix(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrixViaInverseDynamics(*context_, &mass_matrix);

  MatrixX<double> mass_matrix_ri(plant_ri_.num_velocities(),
                                 plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrixViaInverseDynamics(*context_ri_, &mass_matrix_ri);

  // Compute a suitable tolerance scaled with the norm of the mass matrix.
  // Since .norm() computes the Frobenius norm, and num_velocities() is the
  // squared root of the number of elements in the matrix, this tolerance is
  // effectively being scaled by the RMS value of the elements in the mass
  // matrix.
  const double kTolerance = 16.0 * std::numeric_limits<double>::epsilon() *
                            mass_matrix.norm() / plant_.num_velocities();

  // The difference between the mass matrix of the system with reflected inertia
  // and the system without is a diagonal matrix whose diagonal elements are the
  // reflected inertias.
  const MatrixX<double> Mdiff = mass_matrix_ri - mass_matrix;
  const MatrixX<double> Mdiagonal =
      reflected_inertia_with_generalized_velocity_indices.asDiagonal();
  EXPECT_TRUE(CompareMatrices(Mdiff, Mdiagonal, kTolerance,
                              MatrixCompareType::relative));
}

// This test verifies that reflected inertia properly propagates to
// MultibodyPlant::CalcKineticEnergy() by comparing that calculation with
// the kinetic energy calculated using the mass matrix. It also verifies that
// reflected inertia has NO effect on
// MultibodyPlant::CalcSpatialMomentumInWorldAboutPoint() by comparing the
// results from the plant modeled with and the plant modeled without reflected
// inertia.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcKineticEnergyAndMomentum) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Compute the system's kinetic energy with 0.5 * vᵀ M v.
  MatrixX<double> mass_matrix_ri(plant_ri_.num_velocities(),
                                 plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrix(*context_ri_, &mass_matrix_ri);

  const auto& v_ri = plant_ri_.GetVelocities(*context_ri_);

  double energy_mass_matrix = 0.5 * v_ri.transpose() * mass_matrix_ri * v_ri;

  // Have the plant compute its kinetic energy.
  double energy_plant = plant_ri_.CalcKineticEnergy(*context_ri_);

  const double kTolerance = 16.0 * std::numeric_limits<double>::epsilon();

  EXPECT_NEAR(energy_mass_matrix, energy_plant, kTolerance);

  // Verify that the spatial momentum for plant_ and plant_ri are equal.
  // Reminder: Although kinetic energy and mass matrix account for reflected
  // inertia, angular momentum does not account for reflected inertia as it
  // depends on internal mechanics of the gear (e.g., number of gear stages).

  // Form the systems' spatial momentum in world W about Wo, expressed in W.
  const Vector3<double> p_WoWo_W = Vector3<double>::Zero();
  const SpatialMomentum<double> L_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, p_WoWo_W);
  const SpatialMomentum<double> L_W_ri =
      plant_ri_.CalcSpatialMomentumInWorldAboutPoint(*context_ri_, p_WoWo_W);
  EXPECT_TRUE(
      CompareMatrices(L_W.get_coeffs(), L_W_ri.get_coeffs(), kTolerance));
}

// This test verifies the expected difference in the output from
// MultibodyPlant::CalcInverseDynamics() between the models
// with and without reflected inertia.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcInverseDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Map reflected inertias from actuation indices to generalized velocity
  // indices.
  const VectorX<double> reflected_inertia_with_generalized_velocity_indices =
      MapVectorFromActuatorIndicesToGeneralizedVelocityIndices(
          plant_, reflected_inertia_with_actuator_indices);

  // Dynamic equations of motion for the plant_ and plant_ri have the form
  // M(q) v̇ + C(q, v) v - tau_applied = τ where M is mass matrix for plant_.
  // M̃(q) v̇ + C(q, v) v - tau_applied = τ̃ where M̃ is the mass matrix for
  // plant_ri, where τ can be calculated by plant_.CalcInverseDynamics() and τ̃
  // can be calculated by plant_ri_.CalcInverseDynamics(). Since reflected
  // inertia should not affect either C(q,v) or tau_applied, the 2nd equation
  // subtracted from the 1st equation gives (M̃ - M) v̇ = τ - τ̃. Setting v̇
  // to 𝟏 leads to τ - τ̃ = (M̃ - M) [𝟏] , which are exactly the values in
  // reflected_inertia_with_generalized_velocity_indices.
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

  const double kTolerance = 16.0 * std::numeric_limits<double>::epsilon();

  VectorX<double> tau_diff = tau_ri - tau;
  EXPECT_TRUE(CompareMatrices(
      tau_diff, reflected_inertia_with_generalized_velocity_indices, kTolerance,
      MatrixCompareType::relative));
}

// This test verifies that forward and inverse dynamics are consistent by
// propagating the output of forward dynamics through inverse dynamics and
// checking that the resulting generalized forces match the input.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcForwardAndInverseDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Set actuation forces to random values.
  const VectorX<double> tau_arm =
      VectorX<double>::Random(plant_ri_.num_actuated_dofs(arm_model));
  const VectorX<double> tau_gripper =
      VectorX<double>::Random(plant_ri_.num_actuated_dofs(gripper_model));
  plant_ri_.get_actuation_input_port(arm_model).FixValue(context_ri_.get(),
                                                         tau_arm);
  plant_ri_.get_actuation_input_port(gripper_model)
      .FixValue(context_ri_.get(), tau_gripper);

  // Map tau_expected from actuation indices to generalized velocity indices.
  VectorX<double> tau_expected(tau_arm.size() + tau_gripper.size());
  tau_expected << tau_arm, tau_gripper;
  tau_expected = MapVectorFromActuatorIndicesToGeneralizedVelocityIndices(
      plant_, tau_expected);

  // Dynamic equations of motion for the plant_ri have the form
  // M̃(q) v̇ + C(q, v) v - tau_applied = τ̃
  // In step #1, τ̃ is specified and v̇ is calculated (forward dynamics).
  VectorX<double> vdot = plant_ri_.get_generalized_acceleration_output_port()
                             .Eval<systems::BasicVector<double>>(*context_ri_)
                             .CopyToVector();

  // In step #2, v̇ is specified and τ̃ is calculated (inverse dynamics)
  MultibodyForces<double> forces(plant_);
  plant_ri_.CalcForceElementsContribution(*context_ri_, &forces);
  const VectorX<double> tau_inverse_dynamics =
      plant_ri_.CalcInverseDynamics(*context_ri_, vdot, forces);

  // Compare tau_inverse_dynamics to tau_expected. Since these elements
  // have a magnitude near 1 and since the calculation involved in generating
  // these values is fairly involved (many additions, multiplications, etc.), we
  // allow 6 bits of error (2^6 = 64) in the difference between these values.
  const double kTolerance = 64.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(tau_inverse_dynamics, tau_expected, kTolerance,
                              MatrixCompareType::relative));
}

// Dynamic equations of motion for the plant_ri have the form
// M̃(q) v̇ + C(q, v) v - tau_applied = τ̃ Note: In this test τ̃ = 0.
// This test calculates v̇ (forward dynamics) with two different algorithms,
// namely the Articulated Body Algorithm and by inverting the mass matrix M̃.
TEST_F(MultibodyPlantReflectedInertiaTests, CalcForwardDynamics) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Set actuation forces to 0.
  const VectorX<double> tau_arm =
      VectorX<double>::Zero(plant_ri_.num_actuated_dofs(arm_model));
  const VectorX<double> tau_gripper =
      VectorX<double>::Zero(plant_ri_.num_actuated_dofs(gripper_model));
  plant_ri_.get_actuation_input_port(arm_model).FixValue(context_ri_.get(),
                                                         tau_arm);
  plant_ri_.get_actuation_input_port(gripper_model)
      .FixValue(context_ri_.get(), tau_gripper);

  VectorX<double> vdot_ABA =
      plant_ri_.get_generalized_acceleration_output_port()
          .Eval<systems::BasicVector<double>>(*context_ri_)
          .CopyToVector();

  // Calculate v̇ by explicitly solving forward dynamics as:
  // M̃(q) v̇ + C(q, v) v - tau_applied = 0 ,
  // v̇ = - M̃⁻¹ • ( C(q, v) v - tau_applied )
  // = - M̃⁻¹ • tau_etc where tau_etc = C(q, v) v - tau_applied.
  const int nv = plant_ri_.num_velocities();
  MatrixX<double> mass_matrix(nv, nv);
  plant_ri_.CalcMassMatrixViaInverseDynamics(*context_ri_, &mass_matrix);

  // Compute force element contributions.
  MultibodyForces<double> forces(plant_);
  plant_ri_.CalcForceElementsContribution(*context_ri_, &forces);

  // Use inverse dynamics to form tau_etc = C(q, v) v - tau_applied.
  // To see this, notice that when v̇ = 0 is plugged into the equation
  // M̃(q) v̇ + C(q, v) v - tau_applied = tau_etc, it simplifies to
  // M̃(q) [0] + C(q, v) v - tau_applied = tau_etc
  const VectorX<double> zero_vdot = VectorX<double>::Zero(nv);
  const VectorX<double> tau_etc =
      plant_ri_.CalcInverseDynamics(*context_ri_, zero_vdot, forces);

  // Solve for vdot explicitly.
  const VectorX<double> vdot = -(mass_matrix.llt().solve(tau_etc));

  // We estimate the difference between vdot and vdot_ABA to be in the order of
  // machine epsilon times the condition number "kappa" of the mass matrix.
  const double mass_matrix_condition_number = 1.0 / mass_matrix.llt().rcond();
  const double kTolerance =
      mass_matrix_condition_number * std::numeric_limits<double>::epsilon();

  EXPECT_TRUE(
      CompareMatrices(vdot_ABA, vdot, kTolerance, MatrixCompareType::relative));
}

// This test verfies that reflected inertia values are propagated through system
// scalar conversion.
TEST_F(MultibodyPlantReflectedInertiaTests, ScalarConversion) {
  // Arbitrary reflected inertia values.
  VectorX<double> rotor_inertias(kNumJoints);
  VectorX<double> gear_ratios(kNumJoints);
  rotor_inertias << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9;
  gear_ratios << 1, 2, 4, 8, 16, 32, 64, 128, 256;

  const VectorX<double> reflected_inertia_with_actuator_indices =
      gear_ratios.array() * gear_ratios.array() * rotor_inertias.array();

  // Load the models.
  LoadBothModelsSetStateAndFinalize(rotor_inertias, gear_ratios);

  // Compute mass matrix of the original system.
  MatrixX<double> mass_matrix_ri(plant_ri_.num_velocities(),
                                 plant_ri_.num_velocities());
  plant_ri_.CalcMassMatrix(*context_ri_, &mass_matrix_ri);

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
  const double kTolerance = 16.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(mass_matrix_ri, M_autodiff, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
