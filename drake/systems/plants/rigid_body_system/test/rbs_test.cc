#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/rigid_body_system/rigid_body_system.h"

using std::make_unique;
using std::unique_ptr;
using drake::systems::RigidBodySystem;

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
  // Instantiates an empty rigid body system with only a "world" body.
  auto rigid_body_sys = make_unique<RigidBodySystem<double>>();

  // Adds a URDF to the rigid body system. This URDF contains only fixed joints
  // and is attached to the world via a fixed joint. Thus, everything in the
  // URDF becomes part of the world.
  rigid_body_sys->AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
      "/systems/plants/rigid_body_system/test/world.urdf", DrakeJoint::FIXED);

  // Verifies that the number of states, inputs, and outputs are all zero.
  EXPECT_EQ(rigid_body_sys->get_num_states(), 0);
  EXPECT_EQ(rigid_body_sys->get_num_inputs(), 0);
  EXPECT_EQ(rigid_body_sys->get_num_outputs(), 0);

  // Obtains a const pointer to the underlying multibody world in the system.
  const RigidBodyTree& mbd_world = rigid_body_sys->get_multibody_world();

  (void) mbd_world;

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
    // Instantiate a rigid body system and load a Kuka arm model from a URDF
    // file.
    kuka_system_ = make_unique<RigidBodySystem<double>>();
    kuka_system_->AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        DrakeJoint::FIXED);
    // Pointer to the abstract system type used to access System<T> methods not
    // accessible to the users of RigidBodySystem<T>.
    system_ = kuka_system_.get();

    context_ = system_->CreateDefaultContext();
    output_ = system_->AllocateOutput(*context_);

    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  unique_ptr<RigidBodySystem<double>> kuka_system_;
  System<double>* system_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(KukaArmTest, EvalOutput) {
  // Checks that the number of input and output ports in the system and context
  // are consistent.
  ASSERT_EQ(1, kuka_system_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());

  // Checks the size of the input ports to match the number of generalized
  // forces that can be applied.
  ASSERT_EQ(7, kuka_system_->get_num_generalized_forces());
  ASSERT_EQ(7, kuka_system_->get_input_port(0).get_size());

  // Connect to a "fake" free standing input.
  //std::unique_ptr<BasicVector<double>> input_vector;
  context_->SetInputPort(0, MakeInput(
      make_unique<BasicVector<double>>(
          kuka_system_->get_num_generalized_forces())));

  // This call should not assert when compiling Debug builds.
  system_->EvalOutput(*context_, output_.get());

}

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

}  // namespace
}  // namespace test
}  // namespace rigid_body_system
}  // namespace plants
}  // namespace systems
}  // namespace drake
