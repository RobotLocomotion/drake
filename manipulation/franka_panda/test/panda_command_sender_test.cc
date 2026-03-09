#include "drake/manipulation/franka_panda/panda_command_sender.h"

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace franka_panda {

using drake::lcmt_panda_command;
using drake::lcmt_panda_status;
using drake::systems::Context;
using drake::test::LimitMalloc;
using Eigen::VectorXd;

constexpr int N = kPandaArmNumJoints;

class PandaCommandSenderTest : public testing::Test {
 public:
  PandaCommandSenderTest()
      : dut_(N, PandaControlModes::kPosition | PandaControlModes::kTorque),
        context_ptr_(dut_.CreateDefaultContext()),
        context_(*context_ptr_) {}

  const lcmt_panda_command& output() const {
    return dut_.get_output_port().Eval<lcmt_panda_command>(context_);
  }

 protected:
  PandaCommandSender dut_;
  std::unique_ptr<Context<double>> context_ptr_;
  Context<double>& context_;

  const VectorXd q0_{VectorXd::LinSpaced(N, 0.1, 0.2)};
  const std::vector<double> std_q0_{q0_.data(), q0_.data() + q0_.size()};

  const VectorXd v0_{VectorXd::LinSpaced(N, 0.2, 0.3)};
  const std::vector<double> std_v0_{v0_.data(), v0_.data() + v0_.size()};

  const VectorXd t0_{VectorXd::LinSpaced(N, 0.3, 0.4)};
  const std::vector<double> std_t0_{t0_.data(), t0_.data() + t0_.size()};
};

TEST_F(PandaCommandSenderTest, PositionAndTorque) {
  dut_.get_position_input_port().FixValue(&context_, q0_);
  dut_.get_torque_input_port().FixValue(&context_, t0_);
  EXPECT_EQ(output().control_mode_expected,
            to_int(PandaControlModes::kPosition | PandaControlModes::kTorque));
  EXPECT_EQ(output().num_joint_position, N);
  EXPECT_EQ(output().joint_position, std_q0_);
  EXPECT_EQ(output().num_joint_velocity, 0);
  EXPECT_EQ(output().joint_velocity.size(), 0);
  EXPECT_EQ(output().num_joint_torque, N);
  EXPECT_EQ(output().joint_torque, std_t0_);
}

TEST_F(PandaCommandSenderTest, PositionOnly) {
  const PandaCommandSender sender(N, PandaControlModes::kPosition);
  auto context = sender.CreateDefaultContext();
  sender.get_position_input_port().FixValue(context.get(), q0_);
  const lcmt_panda_command output =
      sender.get_output_port().Eval<lcmt_panda_command>(*context);
  EXPECT_EQ(output.control_mode_expected, to_int(PandaControlModes::kPosition));
  EXPECT_EQ(output.num_joint_position, N);
  EXPECT_EQ(output.joint_position, std_q0_);
  EXPECT_EQ(output.num_joint_velocity, 0);
  EXPECT_EQ(output.joint_velocity.size(), 0);
  EXPECT_EQ(output.num_joint_torque, 0);
  EXPECT_EQ(output.joint_torque.size(), 0);
}

TEST_F(PandaCommandSenderTest, VelocityOnly) {
  const PandaCommandSender sender(N, PandaControlModes::kVelocity);
  auto context = sender.CreateDefaultContext();
  sender.get_velocity_input_port().FixValue(context.get(), v0_);
  const lcmt_panda_command output =
      sender.get_output_port().Eval<lcmt_panda_command>(*context);
  EXPECT_EQ(output.control_mode_expected, to_int(PandaControlModes::kVelocity));
  EXPECT_EQ(output.num_joint_position, 0);
  EXPECT_EQ(output.joint_position.size(), 0);
  EXPECT_EQ(output.num_joint_velocity, N);
  EXPECT_EQ(output.joint_velocity, std_v0_);
  EXPECT_EQ(output.num_joint_torque, 0);
  EXPECT_EQ(output.joint_torque.size(), 0);
}

TEST_F(PandaCommandSenderTest, TorqueOnly) {
  const PandaCommandSender sender(N, PandaControlModes::kTorque);
  auto context = sender.CreateDefaultContext();
  sender.get_torque_input_port().FixValue(context.get(), t0_);
  const lcmt_panda_command output =
      sender.get_output_port().Eval<lcmt_panda_command>(*context);
  EXPECT_EQ(output.control_mode_expected, to_int(PandaControlModes::kTorque));
  EXPECT_EQ(output.num_joint_position, 0);
  EXPECT_EQ(output.joint_position.size(), 0);
  EXPECT_EQ(output.num_joint_velocity, 0);
  EXPECT_EQ(output.joint_velocity.size(), 0);
  EXPECT_EQ(output.num_joint_torque, N);
  EXPECT_EQ(output.joint_torque, std_t0_);
}

// This class is likely to be used on the critical path for robot control, so
// we insist that it must not perform heap operations while in steady-state.
TEST_F(PandaCommandSenderTest, MallocTest) {
  // Compute the output once; this will allocate all required storage.
  auto& q = dut_.get_position_input_port().FixValue(&context_, q0_);
  auto& tau = dut_.get_torque_input_port().FixValue(&context_, t0_);
  output();

  // Change q and recompute; no heap changes are allowed.
  {
    LimitMalloc guard;
    q.GetMutableVectorData<double>();
    output();
  }

  // Change tau and recompute; no heap changes are allowed.
  {
    LimitMalloc guard;
    tau.GetMutableVectorData<double>();
    output();
  }
}

TEST_F(PandaCommandSenderTest, BadControlMode) {
  EXPECT_THROW(PandaCommandSender(7, static_cast<PandaControlMode>(0x9999)),
               std::exception);
}

}  // namespace franka_panda
}  // namespace manipulation
}  // namespace drake
