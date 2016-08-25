#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_system/rigid_body_system.h"

using drake::parsers::ModelInstanceIdTable;
using drake::systems::RigidBodySystem;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_system {
namespace test {
namespace {

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Tests the ability to load a URDF as part of the world of a rigid body system.
GTEST_TEST(RigidBodySystemTest, TestLoadURDFWorld) {
  // Instantiates an MBD model of the world.
  auto mbd_world_ptr = make_unique<RigidBodyTree>();
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() +
          "/systems/plants/rigid_body_system/test/world.urdf",
          DrakeJoint::FIXED, nullptr /* weld to frame */, mbd_world_ptr.get());

  // Instantiates a RigidBodyPlant from an MBD model of the world.
  RigidBodySystem<double> rigid_body_sys(move(mbd_world_ptr));

  // Verifies that the number of states, inputs, and outputs are all zero.
  EXPECT_EQ(rigid_body_sys.get_num_states(), 0);
  EXPECT_EQ(rigid_body_sys.get_num_inputs(), 0);
  EXPECT_EQ(rigid_body_sys.get_num_outputs(), 0);

  // Obtains a const pointer to the underlying multibody world in the system.
  const RigidBodyTree& mbd_world = rigid_body_sys.get_multibody_world();

  // Checks that the bodies in the multibody world can be obtained by name and
  // that they have the correct model name.
  for (auto& body_name :
       {"floor", "ramp_1", "ramp_2", "box_1", "box_2", "box_3", "box_4"}) {
    RigidBody* body = mbd_world.FindBody(body_name);
    EXPECT_NE(body, nullptr);
    EXPECT_EQ(body->get_model_name(), "dual_ramps");
  }
}

class KukaArmTest : public ::testing::Test {
 protected:
  void SetUp() override {

    // Instantiates an MBD model of the world.
    auto mbd_world = make_unique<RigidBodyTree>();
    ModelInstanceIdTable model_instance_id_table =
        drake::parsers::urdf::AddModelInstanceFromUrdfFile(
            drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
            DrakeJoint::FIXED, nullptr /* weld to frame */, mbd_world.get());

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    kuka_system_ = make_unique<RigidBodySystem<double>>(move(mbd_world));

    // Pointer to the abstract system type used to access System<T> methods not
    // accessible to the users of RigidBodySystem<T>.
    system_ = kuka_system_.get();

    context_ = system_->CreateDefaultContext();
    output_ = system_->AllocateOutput(*context_);
    derivatives_ = system_->AllocateTimeDerivatives();
  }

  const int kNumPositions_{7};
  const int kNumVelocities_{7};
  const int kNumStates_{kNumPositions_ + kNumVelocities_};

  unique_ptr<RigidBodySystem<double>> kuka_system_;
  System<double>* system_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
};

// Tests that the KukaArm system allocates in the context_ a continuous state
// of the proper size.
TEST_F(KukaArmTest, StateHasTheRightSizes) {
  const StateVector<double>& xc =
      context_->get_state().continuous_state->get_generalized_position();
  const StateVector<double>& vc =
      context_->get_state().continuous_state->get_generalized_velocity();
  const StateVector<double>& zc =
      context_->get_state().continuous_state->get_misc_continuous_state();

  EXPECT_EQ(7, xc.size());
  EXPECT_EQ(7, vc.size());
  EXPECT_EQ(0, zc.size());
}

TEST_F(KukaArmTest, ObtainZeroConfiguration) {
  // Connect to a "fake" free standing input.
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_actuators())));

  kuka_system_->ObtainZeroConfiguration(context_.get());

  // Asserts that for this case the zero configuration corresponds to a state
  // vector with all entries equal to zero.
  VectorXd xc = context_->get_xc().CopyToVector();
  ASSERT_EQ(14, xc.size());
  ASSERT_EQ(xc, VectorXd::Zero(xc.size()));
}

TEST_F(KukaArmTest, EvalOutput) {

  // Checks that the number of input and output ports in the system and context
  // are consistent.
  ASSERT_EQ(1, kuka_system_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());

  // Checks the size of the input ports to match the number of generalized
  // forces that can be applied.
  ASSERT_EQ(7, kuka_system_->get_num_generalized_positions());
  ASSERT_EQ(7, kuka_system_->get_num_generalized_velocities());
  ASSERT_EQ(14, kuka_system_->get_num_states());
  ASSERT_EQ(7, kuka_system_->get_num_actuators());
  ASSERT_EQ(7, kuka_system_->get_input_port(0).get_size());

  // Connect to a "fake" free standing input.
  //std::unique_ptr<BasicVector<double>> input_vector;
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_actuators())));

  // Zeroes the state.
  kuka_system_->ObtainZeroConfiguration(context_.get());

  // Sets the state to a non-zero value.
  VectorXd desired_angles(kNumPositions_);
  desired_angles << 0.5, 0.1, -0.1, 0.2, 0.3, -0.2, 0.15;
  for (int i = 0; i < kNumPositions_; ++i) {
    kuka_system_->set_position(context_.get(), i, desired_angles[i]);
  }
  VectorXd desired_state(kNumStates_);
  desired_state << desired_angles, VectorXd::Zero(kNumVelocities_);
  VectorXd xc = context_->get_xc().CopyToVector();
  ASSERT_EQ(xc, desired_state);

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port =
      dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output_port);

  // This call should not assert when compiling Debug builds.
  system_->EvalOutput(*context_, output_.get());

  // Asserts the output equals the state.
  EXPECT_EQ(desired_state, output_port->get_value());

}

TEST_F(KukaArmTest, EvalTimeDerivatives) {

  // Connect to a "fake" free standing input.
  //std::unique_ptr<BasicVector<double>> input_vector;
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_actuators())));

  // Zeroes the state.
  kuka_system_->ObtainZeroConfiguration(context_.get());

  // Sets the state to a non-zero value.
  VectorXd desired_angles(kNumPositions_);
  desired_angles << 0.5, 0.1, -0.1, 0.2, 0.3, -0.2, 0.15;
  for (int i = 0; i < kNumPositions_; ++i) {
    kuka_system_->set_position(context_.get(), i, desired_angles[i]);
  }
  VectorXd desired_state(kNumStates_);
  desired_state << desired_angles, VectorXd::Zero(kNumVelocities_);
  VectorXd xc = context_->get_xc().CopyToVector();
  ASSERT_EQ(xc, desired_state);

  ASSERT_EQ(1, output_->get_num_ports());
  const BasicVector<double>* output_port =
      dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
  ASSERT_NE(nullptr, output_port);

  // This call should not assert when compiling Debug builds.
  system_->EvalOutput(*context_, output_.get());

  EXPECT_NO_THROW(system_->EvalTimeDerivatives(*context_, derivatives_.get()));

  // Asserts the output equals the state.
  EXPECT_EQ(desired_state, output_port->get_value());

}


}  // namespace
}  // namespace test
}  // namespace rigid_body_system
}  // namespace plants
}  // namespace systems
}  // namespace drake
