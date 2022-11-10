#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/util/make_arm_controller_model.h"
#include "drake/multibody/parsing/model_instance_info.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

using Eigen::VectorXd;
using math::RigidTransformd;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::parsing::ModelInstanceInfo;
using systems::ConstantVectorSource;
using systems::DiagramBuilder;
using systems::SharedPointerSystem;

constexpr int N = manipulation::kuka_iiwa::kIiwaArmNumJoints;
constexpr double kTolerance = 1e-3;

class BuildIiwaControlTest : public ::testing::Test {
 public:
  BuildIiwaControlTest() = default;

 protected:
  void SetUp() {
    sim_plant_ = builder_.AddSystem<MultibodyPlant<double>>(0.001);
    Parser parser{sim_plant_};
    const std::string iiwa7_model_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7"
        "/iiwa7_no_collision.sdf");
    const ModelInstanceIndex iiwa7_instance =
        parser.AddModels(iiwa7_model_path).at(0);
    const std::string iiwa7_model_name =
        sim_plant_->GetModelInstanceName(iiwa7_instance);

    iiwa7_info_.model_name = iiwa7_model_name;
    iiwa7_info_.model_path = iiwa7_model_path;
    iiwa7_info_.child_frame_name = "iiwa_link_0";
    iiwa7_info_.model_instance = iiwa7_instance;

    // Weld the arm to the world frame.
    sim_plant_->WeldFrames(
        sim_plant_->world_frame(),
        sim_plant_->GetFrameByName("iiwa_link_0", iiwa7_instance),
        RigidTransformd::Identity());
    sim_plant_->Finalize();

    controller_plant_ = SharedPointerSystem<double>::AddToBuilder(
        &builder_, internal::MakeArmControllerModel(*sim_plant_, iiwa7_info_));
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* sim_plant_{nullptr};
  MultibodyPlant<double>* controller_plant_{nullptr};
  lcm::DrakeLcm lcm_;
  ModelInstanceInfo iiwa7_info_;
};

TEST_F(BuildIiwaControlTest, BuildIiwaControl) {
  BuildIiwaControl(*sim_plant_, iiwa7_info_.model_instance, *controller_plant_,
                   &lcm_, &builder_);
  const auto diagram = builder_.Build();
  systems::Simulator<double> simulator(*diagram);

  lcm::Subscriber<lcmt_iiwa_status> sub{&lcm_, "IIWA_STATUS"};

  // Publish commands and check whether the Iiwa arm moves to the correct poses.
  lcmt_iiwa_command command{};
  const std::vector<double> q1{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  command.num_joints = N;
  command.joint_position = q1;
  Publish(&lcm_, "IIWA_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(1.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_EQ(sub.message().joint_position_commanded.size(), N);
  EXPECT_EQ(sub.message().joint_position_commanded, q1);
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(sub.message().joint_position_measured[i], q1[i], kTolerance);
  }

  const std::vector<double> q2(N, 0.0);
  command.joint_position = q2;
  Publish(&lcm_, "IIWA_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(2.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_EQ(sub.message().joint_position_commanded.size(), N);
  EXPECT_EQ(sub.message().joint_position_commanded, q2);
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(sub.message().joint_position_measured[i], q2[i], kTolerance);
  }
}

TEST_F(BuildIiwaControlTest, FeedforwardDisabled) {
  IiwaControlPorts control_ports{};
  control_ports = BuildSimplifiedIiwaControl(
      *sim_plant_, iiwa7_info_.model_instance, *controller_plant_, &builder_,
      0.01, {}, kIiwaPositionMode);
  const auto diagram = builder_.Build();

  EXPECT_NE(control_ports.commanded_positions, nullptr);
  EXPECT_EQ(control_ports.commanded_positions->size(), N);
  // Should be nullptr if enable_feedforward_torque is false.
  EXPECT_EQ(control_ports.commanded_torque, nullptr);
  EXPECT_NE(control_ports.joint_torque, nullptr);
  EXPECT_EQ(control_ports.joint_torque->size(), N);
  EXPECT_NE(control_ports.external_torque, nullptr);
  EXPECT_EQ(control_ports.external_torque->size(), N);
}

TEST_F(BuildIiwaControlTest, TorqueOnly) {
  IiwaControlPorts control_ports{};
  control_ports = BuildSimplifiedIiwaControl(
      *sim_plant_, iiwa7_info_.model_instance, *controller_plant_, &builder_,
      0.01, {}, kIiwaTorqueMode);
  const auto diagram = builder_.Build();

  EXPECT_EQ(control_ports.commanded_positions, nullptr);
  EXPECT_NE(control_ports.commanded_torque, nullptr);
  EXPECT_EQ(control_ports.commanded_torque->size(), N);
  EXPECT_NE(control_ports.joint_torque, nullptr);
  EXPECT_EQ(control_ports.joint_torque->size(), N);
  EXPECT_NE(control_ports.external_torque, nullptr);
  EXPECT_EQ(control_ports.external_torque->size(), N);
}

TEST_F(BuildIiwaControlTest, FeedforwardEnabled) {
  IiwaControlPorts control_ports{};
  const double ext_joint_filter_tau{0.01};
  control_ports = BuildSimplifiedIiwaControl(
      *sim_plant_, iiwa7_info_.model_instance, *controller_plant_, &builder_,
      ext_joint_filter_tau, {});

  /* Send a non-zero position command and a zero-feedforward-torque command. The
   Iiwa arm should reach the commanded position without a problem. */
  VectorXd position_command = VectorXd::LinSpaced(N, 0.3, 0.4);
  auto p_input_source =
      builder_.AddSystem<ConstantVectorSource<double>>(position_command);
  builder_.Connect(p_input_source->get_output_port(),
                   *control_ports.commanded_positions);

  VectorXd zero_feedforward_command = VectorXd::Zero(N);
  auto torque_input_source = builder_.AddSystem<ConstantVectorSource<double>>(
      zero_feedforward_command);
  builder_.Connect(torque_input_source->get_output_port(),
                   *control_ports.commanded_torque);
  auto diagram = builder_.Build();

  // Check all the ports of `control_ports` are set up properly.
  EXPECT_NE(control_ports.commanded_positions, nullptr);
  EXPECT_EQ(control_ports.commanded_positions->size(), N);
  EXPECT_NE(control_ports.commanded_torque, nullptr);
  EXPECT_EQ(control_ports.commanded_torque->size(), N);
  EXPECT_NE(control_ports.joint_torque, nullptr);
  EXPECT_EQ(control_ports.joint_torque->size(), N);
  EXPECT_NE(control_ports.external_torque, nullptr);
  EXPECT_EQ(control_ports.external_torque->size(), N);

  systems::Simulator<double> simulator(*diagram);
  auto& diagram_context = simulator.get_mutable_context();

  simulator.AdvanceTo(1.0);
  VectorXd iiwa_positions = sim_plant_->GetPositions(
      sim_plant_->GetMyContextFromRoot(diagram_context),
      iiwa7_info_.model_instance);
  EXPECT_EQ(iiwa_positions.size(), N);
  for (int i = 0; i < N; ++i) {
    EXPECT_NEAR(iiwa_positions(i), position_command(i), kTolerance);
  }

  /* Same position command with a non-zero feedforward torque. The extra torque
   should take effect and move the Iiwa arm from its commanded position. */
  VectorXd nonzero_feedforward_command = VectorXd::Constant(N, 5.0);
  torque_input_source
      ->get_mutable_source_value(
          &torque_input_source->GetMyMutableContextFromRoot(&diagram_context))
      .set_value(nonzero_feedforward_command);
  simulator.AdvanceTo(2.0);
  iiwa_positions = sim_plant_->GetPositions(
      sim_plant_->GetMyContextFromRoot(diagram_context),
      iiwa7_info_.model_instance);
  for (int i = 0; i < N; ++i) {
    EXPECT_TRUE(std::abs(iiwa_positions(i) - position_command(i)) > kTolerance);
  }
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
