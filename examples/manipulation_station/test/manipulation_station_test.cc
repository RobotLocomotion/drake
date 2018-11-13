#include "drake/examples/manipulation_station/manipulation_station.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using Eigen::Vector2d;
using Eigen::VectorXd;
using multibody::RevoluteJoint;
using systems::BasicVector;

GTEST_TEST(ManipulationStationTest, CheckPlantBasics) {
  ManipulationStation<double> station(0.001);
  station.AddCupboard();
  multibody::parsing::AddModelFromSdfFile(
      FindResourceOrThrow("drake/examples/manipulation_station/models"
                          "/061_foam_brick.sdf"),
      "object", &station.get_mutable_multibody_plant(),
      &station.get_mutable_scene_graph());
  station.Finalize();

  auto& plant = station.get_multibody_plant();
  EXPECT_EQ(plant.num_actuated_dofs(), 9);  // 7 iiwa + 2 wsg.

  auto context = station.CreateDefaultContext();
  auto& plant_context = station.GetSubsystemContext(plant, *context);
  VectorXd q = VectorXd::LinSpaced(7, 0.1, 0.7),
           v = VectorXd::LinSpaced(7, 1.1, 1.7),
           q_command = VectorXd::LinSpaced(7, 2.1, 2.7),
           tau_ff = VectorXd::LinSpaced(7, 3.1, 3.7);

  // Set positions and read them back out, multiple ways.
  station.SetIiwaPosition(q, context.get());
  EXPECT_TRUE(CompareMatrices(q, station.GetIiwaPosition(*context)));
  EXPECT_TRUE(CompareMatrices(q, station.GetOutputPort("iiwa_position_measured")
                                     .Eval<BasicVector<double>>(*context)
                                     .get_value()));
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(q(i), plant
                        .template GetJointByName<RevoluteJoint>(
                            "iiwa_joint_" + std::to_string(i + 1))
                        .get_angle(plant_context));
  }

  // Set velocities and read them back out, multiple ways.
  station.SetIiwaVelocity(v, context.get());
  EXPECT_TRUE(CompareMatrices(v, station.GetIiwaVelocity(*context)));
  EXPECT_TRUE(
      CompareMatrices(v, station.GetOutputPort("iiwa_velocity_estimated")
                             .Eval<BasicVector<double>>(*context)
                             .get_value()));
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(v(i), plant
                        .template GetJointByName<RevoluteJoint>(
                            "iiwa_joint_" + std::to_string(i + 1))
                        .get_angular_rate(plant_context));
  }

  // Check position command pass through.
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        q_command);
  EXPECT_TRUE(CompareMatrices(q_command,
                              station.GetOutputPort("iiwa_position_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check feedforward_torque command.
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));
  VectorXd tau_with_no_ff = station.GetOutputPort("iiwa_torque_commanded")
                                .Eval<BasicVector<double>>(*context)
                                .get_value();
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(), tau_ff);
  EXPECT_TRUE(CompareMatrices(tau_with_no_ff + tau_ff,
                              station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check iiwa_torque_commanded == iiwa_torque_measured.
  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              station.GetOutputPort("iiwa_torque_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check that iiwa_torque_external == 0 (no contact).
  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_external")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero());

  // Check that the additional output ports exist and are spelled correctly.
  EXPECT_NO_THROW(station.GetOutputPort("contact_results"));
  EXPECT_NO_THROW(station.GetOutputPort("plant_continuous_state"));
}

GTEST_TEST(ManipulationStationTest, CheckStateFromPosition) {
  const double kTimeStep = 0.002;
  ManipulationStation<double> station(kTimeStep);
  station.Finalize();

  auto context = station.CreateDefaultContext();

  // Expect state from the velocity interpolators in the iiwa and the wsg and
  // from the multibody state of the plant.
  EXPECT_EQ(context->get_num_discrete_state_groups(), 3);

  // The tests below expect desired_state_from_position to be the second
  // state.  Verify this by checking the sizes.
  const int plant_index = 0, state_from_position_index = 1;
  EXPECT_GT(context->get_discrete_state(plant_index).size(),
            7);  // iiwa+wsg pos+vel.
  const int plant_state_size = context->get_discrete_state(plant_index).size();
  EXPECT_EQ(context->get_discrete_state(state_from_position_index).size(), 7);

  // Expect continuous state from the integral term in the PID from the
  // inverse dynamics controller.
  EXPECT_EQ(context->get_continuous_state().size(), 7);

  const VectorXd iiwa_position = VectorXd::LinSpaced(7, 0.43, 0.7);
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        iiwa_position);
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));
  context->FixInputPort(station.GetInputPort("wsg_position").get_index(),
                        Vector1d(0.05));
  context->FixInputPort(station.GetInputPort("wsg_force_limit").get_index(),
                        Vector1d(40));
  context->get_mutable_discrete_state(plant_index).SetZero();
  context->get_mutable_discrete_state(state_from_position_index).SetZero();

  auto next_state = station.AllocateDiscreteVariables();
  station.CalcDiscreteVariableUpdates(*context, next_state.get());

  EXPECT_TRUE(CompareMatrices(
      next_state->get_vector(state_from_position_index).get_value(),
      iiwa_position));

  // Partially check M(q)vdot ≈ Mₑ(q)vdot_desired + τ_feedforward + τ_external
  // by setting the right side to zero and confirming that vdot ≈ 0.
  const auto& plant = station.get_multibody_plant();
  // Make up some state (with zeros for non-iiwa states).
  VectorXd arbitrary_plant_state = VectorXd::Zero(plant_state_size);
  const auto& base_joint =
      plant.GetJointByName<multibody::RevoluteJoint>("iiwa_joint_1");
  const int iiwa_position_start = base_joint.position_start();
  const int iiwa_velocity_start =
      plant.num_positions() + base_joint.velocity_start();
  arbitrary_plant_state.segment<7>(iiwa_position_start) =
      VectorXd::LinSpaced(7, 0.735, 0.983);
  arbitrary_plant_state.segment<7>(iiwa_velocity_start) =
      VectorXd::LinSpaced(7, -1.23, 0.456);
  context->get_mutable_discrete_state(plant_index)
      .SetFromVector(arbitrary_plant_state);

  // Set desired position to actual position and the desired velocity to the
  // actual velocity.
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        arbitrary_plant_state.segment<7>(iiwa_position_start));
  // Last iiwa_position should have been iiwa_position - time_step*velocity.
  context->get_mutable_discrete_state(state_from_position_index)
      .SetFromVector(arbitrary_plant_state.segment<7>(iiwa_position_start) -
                     kTimeStep *
                         arbitrary_plant_state.segment<7>(iiwa_velocity_start));
  // Set integral terms to zero.
  context->get_mutable_continuous_state_vector().SetZero();

  // Ensure that the feedforward torque is zero.
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));

  // Check that iiwa_torque_external == 0 (no contact).
  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_external")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero());

  station.CalcDiscreteVariableUpdates(*context, next_state.get());

  // Check that vdot ≈ 0 by checking that next velocity ≈ velocity.
  VectorXd vddot = (next_state->get_vector(plant_index)
                        .get_value()
                        .segment<7>(iiwa_velocity_start) -
                    arbitrary_plant_state.segment<7>(iiwa_velocity_start)) /
                   kTimeStep;

  // Note: This tolerance could be 2e-12 if the wsg was not attached.
  // After significant inspection, RussTedrake and amcastro-TRI believe that
  // the remaining source of error is due entirely to the lack of
  // compensation for the dynamics of the fingers.  We verified that using
  // the lumped gripper model "exactly" cancels out the ID torques on the arm
  // joints. However, and especially if velocities are non-zero, nothing
  // prevents the fingers from sliding off the gripper's body. That is, ID is
  // not enough to keep the arm static. Even if by small amount, the fingers
  // will attempt to move, and since all dofs in the arm are coupled, including
  // those of the fingers, we see this error propagated into the other joints
  // as well.
  const double kTolerance = 0.05;  // rad/sec^2.
  EXPECT_TRUE(CompareMatrices(vddot, VectorXd::Zero(7), kTolerance));
}

GTEST_TEST(ManipulationStationTest, CheckWsg) {
  ManipulationStation<double> station(0.001);
  station.Finalize();

  auto context = station.CreateDefaultContext();

  const double q = 0.023;
  const double v = 0.12;

  station.SetWsgPosition(q, context.get());
  EXPECT_EQ(station.GetWsgPosition(*context), q);

  station.SetWsgVelocity(v, context.get());
  EXPECT_EQ(station.GetWsgVelocity(*context), v);

  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("wsg_state_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              Vector2d(q, v)));

  EXPECT_NO_THROW(station.GetOutputPort("wsg_force_measured"));
}

GTEST_TEST(ManipulationStationTest, CheckRGBDOutputs) {
  ManipulationStation<double> station(0.001);
  station.Finalize();

  auto context = station.CreateDefaultContext();

  const int kNumCameras = 3;
  for (int i = 0; i < kNumCameras; i++) {
    // Make sure the camera outputs can be evaluated, and are non-empty.
    EXPECT_GE(station.GetOutputPort("camera" + std::to_string(i) + "_rgb_image")
                  .Eval<systems::sensors::ImageRgba8U>(*context)
                  .size(),
              0);
    EXPECT_GE(
        station.GetOutputPort("camera" + std::to_string(i) + "_depth_image")
            .Eval<systems::sensors::ImageDepth32F>(*context)
            .size(),
        0);
    EXPECT_GE(
        station.GetOutputPort("camera" + std::to_string(i) + "_label_image")
            .Eval<systems::sensors::ImageLabel16I>(*context)
            .size(),
        0);
  }
}

GTEST_TEST(ManipulationStationTest, CheckCollisionVariants) {
  ManipulationStation<double> station1(
      0.002, IiwaCollisionModel::kNoCollision);

  // In this variant, there are collision geometries from the world and the
  // gripper, but not from the iiwa.
  const int num_collisions =
      station1.get_multibody_plant().num_collision_geometries();

  ManipulationStation<double> station2(
      0.002, IiwaCollisionModel::kBoxCollision);
  // Check for additional collision elements (one for each link, which includes
  // the base).
  EXPECT_EQ(station2.get_multibody_plant().num_collision_geometries(),
            num_collisions + 8);

  // The controlled model does not register with a scene graph, so has zero
  // collisions.
  EXPECT_EQ(station2.get_controller_plant().num_collision_geometries(), 0);
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
