#include "drake/systems/sensors/gyroscope.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/gyroscope_output.h"

using Eigen::Vector3d;

using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace sensors {
namespace {

const char* const kSensorName = "GyroscopeName";

class TestGyroscope : public ::testing::Test {
 public:
  TestGyroscope() : tree_(make_unique<RigidBodyTree<double>>()) {}

  void SetUp() override {
    // Adds a box to the RigidBodyTree.
    AddModelInstanceFromUrdfFileToWorld(
        GetDrakePath() + "/multibody/models/box.urdf",
        drake::multibody::joints::kQuaternion, tree_.get());

    // Adds a frame to the RigidBodyTree called "box frame" that is coincident
    // with the "box" body within the RigidBodyTree.
    auto frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box frame",
        tree_->FindBody("box"), Eigen::Isometry3d::Identity());
    tree_->addFrame(frame);
    EXPECT_EQ(tree_->get_num_actuators(), 0);

    // Defines the Device Under Test (DUT).
    dut_ = make_unique<Gyroscope>(kSensorName, *frame, *tree_);

    context_ = dut_->CreateDefaultContext();
    EXPECT_EQ(context_->get_num_input_ports(), 1);
    EXPECT_EQ(context_->get_continuous_state_vector().size(), 0);

    num_positions_ = tree_->get_num_positions();
    num_velocities_ = tree_->get_num_velocities();

    // The RigidBodyTree has 13 DOFs: 7 generalized positions and 6 generalized
    // velocities.
    EXPECT_EQ(num_positions_, 7);
    EXPECT_EQ(num_velocities_, 6);

    num_states_ = num_positions_ + num_velocities_;
  }

  unique_ptr<RigidBodyTree<double>> tree_;
  unique_ptr<Gyroscope> dut_;
  unique_ptr<Context<double>> context_;
  int num_positions_{};
  int num_velocities_{};
  int num_states_{};
};

// Tests that the gyroscope attached to a single rigid body floating in
// space measures zero rotational velocity.
TEST_F(TestGyroscope, TestFreeFall) {
  VectorX<double> state_vector(num_states_);
  state_vector << tree_->getZeroConfiguration(),
                  VectorX<double>::Zero(tree_->get_num_velocities());
  context_->FixInputPort(
      dut_->get_input_port().get_index(),
      make_unique<BasicVector<double>>(state_vector));

  unique_ptr<SystemOutput<double>> output = dut_->AllocateOutput(*context_);
  ASSERT_EQ(output->get_num_ports(), 1);
  dut_->CalcOutput(*context_, output.get());

  const GyroscopeOutput<double>* gyroscope_output =
      dynamic_cast<const GyroscopeOutput<double>*>(output->get_vector_data(0));

  const Vector3d expected_angular_velocity(0, 0, 0);
  EXPECT_TRUE(CompareMatrices(gyroscope_output->get_rotational_velocities(),
                              expected_angular_velocity, 1e-10,
                              MatrixCompareType::absolute));
}

// Tests that the gyroscope can measure non-zero rotational velocities.
TEST_F(TestGyroscope, TestNonZeroRotationalVelocity) {
  const Vector3d angular_velocity(1.927, 7.873, -33.234);
  VectorX<double> state_vector(num_states_);
  state_vector
      << 0,                    // base_x position
         0,                    // base_y position
         0,                    // base_z position
         1,                    // base_qw position
         0,                    // base_qx position
         0,                    // base_qy position
         0,                    // base_qz position
         angular_velocity(0),  // base_wx velocity
         angular_velocity(1),  // base_wy velocity
         angular_velocity(2),  // base_wz velocity
         0,                    // base_vx velocity
         0,                    // base_vy velocity
         0;                    // base_vz velocity

  context_->FixInputPort(
      dut_->get_input_port().get_index(),
      make_unique<BasicVector<double>>(state_vector));

  unique_ptr<SystemOutput<double>> output = dut_->AllocateOutput(*context_);
  ASSERT_EQ(output->get_num_ports(), 1);
  dut_->CalcOutput(*context_, output.get());

  const GyroscopeOutput<double>* gyroscope_output =
      dynamic_cast<const GyroscopeOutput<double>*>(output->get_vector_data(0));

  EXPECT_TRUE(CompareMatrices(gyroscope_output->get_rotational_velocities(),
                              angular_velocity, 1e-10,
                              MatrixCompareType::absolute));

  // Confirms that Clone is correct.
  std::unique_ptr<BasicVector<double>> cloned_base = gyroscope_output->Clone();
  const GyroscopeOutput<double>* const cloned_sub =
      dynamic_cast<const GyroscopeOutput<double>*>(cloned_base.get());
  ASSERT_NE(cloned_sub, nullptr);
  EXPECT_TRUE(CompareMatrices(cloned_sub->get_rotational_velocities(),
                              angular_velocity, 1e-10,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
