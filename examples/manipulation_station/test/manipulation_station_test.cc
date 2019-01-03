#include "drake/examples/manipulation_station/manipulation_station.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/primitives/discrete_derivative.h"
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
  station.SetupDefaultStation();
  multibody::Parser parser(&station.get_mutable_multibody_plant(),
                           &station.get_mutable_scene_graph());
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/examples/manipulation_station/models"
                          "/061_foam_brick.sdf"),
      "object");
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
  station.SetIiwaPosition(context.get(), q);
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
  station.SetIiwaVelocity(context.get(), v);
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

// Partially check M(q)vdot ≈ Mₑ(q)vdot_desired + τ_feedforward + τ_external
// by setting the right side to zero and confirming that vdot ≈ 0.
GTEST_TEST(ManipulationStationTest, CheckDynamics) {
  const double kTimeStep = 0.002;
  ManipulationStation<double> station(kTimeStep);
  station.SetupDefaultStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();

  // Expect state from the velocity interpolators in the iiwa and the wsg and
  // from the multibody state of the plant.
  EXPECT_EQ(context->get_num_discrete_state_groups(), 3);
  // Expect continuous state from the integral term in the PID from the
  // inverse dynamics controller.
  EXPECT_EQ(context->get_continuous_state().size(), 7);

  const auto& plant = station.get_multibody_plant();

  const VectorXd iiwa_position = VectorXd::LinSpaced(7, 0.735, 0.983);
  const VectorXd iiwa_velocity = VectorXd::LinSpaced(7, -1.23, 0.456);
  station.SetIiwaPosition(context.get(), iiwa_position);
  station.SetIiwaVelocity(context.get(), iiwa_velocity);

  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        iiwa_position);
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(7));
  double wsg_position = station.GetWsgPosition(*context);
  context->FixInputPort(station.GetInputPort("wsg_position").get_index(),
                        Vector1d(wsg_position));
  context->FixInputPort(station.GetInputPort("wsg_force_limit").get_index(),
                        Vector1d(40));

  // Set desired position to actual position and the desired velocity to the
  // actual velocity.
  const auto& position_to_state = dynamic_cast<
      const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
      station.GetSubsystemByName("desired_state_from_position"));
  auto& position_to_state_context =
      station.GetMutableSubsystemContext(position_to_state, context.get());
  position_to_state.set_initial_state(&position_to_state_context, iiwa_position,
                                      iiwa_velocity);
  // Ensure that integral terms are zero.
  context->get_mutable_continuous_state_vector().SetZero();

  // Check that iiwa_torque_external == 0 (no contact).
  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_external")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero());

  auto next_state = station.AllocateDiscreteVariables();
  station.CalcDiscreteVariableUpdates(*context, next_state.get());

  // Check that vdot ≈ 0 by checking that next velocity ≈ velocity.
  const auto& base_joint =
      plant.GetJointByName<multibody::RevoluteJoint>("iiwa_joint_1");
  const int iiwa_velocity_start =
      plant.num_positions() + base_joint.velocity_start();
  VectorXd next_velocity =
      station.GetSubsystemDiscreteValues(plant, *next_state)
          .get_vector()
          .get_value()
          .segment<7>(iiwa_velocity_start);

  // Note: This tolerance could be much smaller if the wsg was not attached.
  const double kTolerance = 1e-4;  // rad/sec.
  EXPECT_TRUE(CompareMatrices(iiwa_velocity, next_velocity, kTolerance));
}

GTEST_TEST(ManipulationStationTest, CheckWsg) {
  ManipulationStation<double> station(0.001);
  station.SetupDefaultStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();

  const double q = 0.023;
  const double v = 0.12;

  station.SetWsgPosition(context.get(), q);
  EXPECT_EQ(station.GetWsgPosition(*context), q);

  station.SetWsgVelocity(context.get(), v);
  EXPECT_EQ(station.GetWsgVelocity(*context), v);

  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("wsg_state_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              Vector2d(q, v)));

  EXPECT_NO_THROW(station.GetOutputPort("wsg_force_measured"));
}

GTEST_TEST(ManipulationStationTest, CheckRGBDOutputs) {
  ManipulationStation<double> station(0.001);
  station.SetupDefaultStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();

  for (const auto& name : station.get_camera_names()) {
    // Make sure the camera outputs can be evaluated, and are non-empty.
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_rgb_image")
                  .Eval<systems::sensors::ImageRgba8U>(*context)
                  .size(),
              0);
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_depth_image")
                  .Eval<systems::sensors::ImageDepth16U>(*context)
                  .size(),
              0);
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_label_image")
                  .Eval<systems::sensors::ImageLabel16I>(*context)
                  .size(),
              0);
  }
}

GTEST_TEST(ManipulationStationTest, CheckCollisionVariants) {
  ManipulationStation<double> station1(0.002);
  station1.SetupDefaultStation(IiwaCollisionModel::kNoCollision);

  // In this variant, there are collision geometries from the world and the
  // gripper, but not from the iiwa.
  const int num_collisions =
      station1.get_multibody_plant().num_collision_geometries();

  ManipulationStation<double> station2(0.002);
  station2.SetupDefaultStation(IiwaCollisionModel::kBoxCollision);
  // Check for additional collision elements (one for each link, which includes
  // the base).
  EXPECT_EQ(station2.get_multibody_plant().num_collision_geometries(),
            num_collisions + 8);

  // The controlled model does not register with a scene graph, so has zero
  // collisions.
  EXPECT_EQ(station2.get_controller_plant().num_collision_geometries(), 0);
}

GTEST_TEST(ManipulationStationTest, SetupClutterClearingStation) {
  ManipulationStation<double> station(0.002);
  station.SetupClutterClearingStation(IiwaCollisionModel::kNoCollision);
  station.Finalize();

  // Make sure we get through the setup and initialization.
  auto context = station.CreateDefaultContext();

  // Check that domain randomization works.
  RandomGenerator generator;
  station.SetRandomContext(context.get(), &generator);
}

// Check that making many stations does not exhaust resources.
GTEST_TEST(ManipulationStationTest, MultipleInstanceTest) {
  for (int i = 0; i < 20; ++i) {
    ManipulationStation<double> station;
    station.SetupDefaultStation();
    station.Finalize();
  }
}

GTEST_TEST(ManipulationStationTest, RegisterRgbdCameraTest) {
  {
    // Test default setup.
    std::map<std::string, math::RigidTransform<double>> default_poses;

    auto set_default_camera_poses = [&default_poses]() {
      default_poses.emplace(
          "0", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(1.69101, 0.176488, 0.432721),
                   Eigen::Vector3d(-0.233066, -0.451461, 0.466761)));

      default_poses.emplace(
          "1", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(-1.68974, 0.20245, -0.706783),
                   Eigen::Vector3d(-0.197236, 0.468471, 0.436499)));

      default_poses.emplace(
          "2", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(0.0438918, 1.03776, -3.13612),
                   Eigen::Vector3d(0.786905, -0.0284378, 1.04287)));
    };

    ManipulationStation<double> dut;
    dut.SetupDefaultStation();

    std::map<std::string, math::RigidTransform<double>> camera_poses =
        dut.GetStaticCameraPosesInWorld();
    set_default_camera_poses();

    EXPECT_EQ(camera_poses.size(), default_poses.size());

    for (const auto& pair : camera_poses) {
      auto found = default_poses.find(pair.first);
      EXPECT_TRUE(found != default_poses.end());
      EXPECT_TRUE(found->second.IsExactlyEqualTo(pair.second));
    }
  }

  {
    // Test registration to custom frames.
    ManipulationStation<double> dut;
    multibody::MultibodyPlant<double>& plant =
        dut.get_mutable_multibody_plant();

    geometry::dev::render::DepthCameraProperties camera_properties(
        640, 480, M_PI_4, geometry::dev::render::Fidelity::kLow, 0.1, 2.0);

    math::RigidTransform<double> X_WF0(Eigen::Vector3d(0, 0, 0.2));
    math::RigidTransform<double> X_F0C0(Eigen::Vector3d(0.3, 0.2, 0.0));
    const auto& frame0 =
        plant.AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
            "frame0", plant.world_frame(), X_WF0.GetAsIsometry3()));
    dut.RegisterRgbdCamera("camera0", frame0, X_F0C0, camera_properties);

    math::RigidTransform<double> X_F0F1(Eigen::Vector3d(0, -0.1, 0.2));
    math::RigidTransform<double> X_F1C1(Eigen::Vector3d(-0.2, 0.2, 0.33));
    const auto& frame1 =
        plant.AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
            "frame1", frame0, X_F0F1.GetAsIsometry3()));
    dut.RegisterRgbdCamera("camera1", frame1, X_F1C1, camera_properties);

    std::map<std::string, math::RigidTransform<double>> camera_poses =
        dut.GetStaticCameraPosesInWorld();

    EXPECT_EQ(camera_poses.size(), 2);
    EXPECT_TRUE(camera_poses.at("camera0").IsExactlyEqualTo(X_WF0 * X_F0C0));
    EXPECT_TRUE(
        camera_poses.at("camera1").IsExactlyEqualTo(X_WF0 * X_F0F1 * X_F1C1));
  }
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
