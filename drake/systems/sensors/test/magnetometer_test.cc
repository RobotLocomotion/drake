#include "drake/systems/sensors/magnetometer.h"

#include <cmath>
#include <limits>
#include <memory>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/magnetometer_output.h"

using Eigen::Vector3d;

using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using math::rpy2quat;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace sensors {
namespace {

const char* const kSensorName = "MagnetometerName";

class TestMagnetometer : public ::testing::Test {
 public:
  TestMagnetometer() : tree_(make_unique<RigidBodyTree<double>>()) {}

  void SetUp() override {
    // Adds a box to the RigidBodyTree and obtains its model instance ID.
    AddModelInstanceFromUrdfFileToWorld(
        GetDrakePath() + "/multibody/models/box.urdf",
        drake::multibody::joints::kQuaternion, tree_.get());

    // Adds a frame to the RigidBodyTree called "sensor frame" that is
    // coincident with the frame of the "box" body within the RigidBodyTree.
    auto frame = std::allocate_shared<RigidBodyFrame<double>>(
        Eigen::aligned_allocator<RigidBodyFrame<double>>(), "sensor frame",
        tree_->FindBody("box"), Eigen::Isometry3d::Identity());
    tree_->addFrame(frame);
    EXPECT_EQ(tree_->get_num_actuators(), 0);

    // Defines the Device Under Test (DUT).
    dut_ = make_unique<Magnetometer>(kSensorName, *frame, *tree_);

    context_ = dut_->CreateDefaultContext();
    EXPECT_EQ(context_->get_num_input_ports(), 1);
    EXPECT_EQ(context_->get_continuous_state_vector().size(), 0);

    output_ = dut_->AllocateOutput(*context_);
    ASSERT_EQ(output_->get_num_ports(), 1);
    magnetometer_output_ = dynamic_cast<const MagnetometerOutput<double>*>(
        output_->get_vector_data(0));
    EXPECT_NE(magnetometer_output_, nullptr);

    num_positions_ = tree_->get_num_positions();
    num_velocities_ = tree_->get_num_velocities();

    // The RigidBodyTree has 13 DOFs: 7 generalized positions and 6 generalized
    // velocities.
    EXPECT_EQ(num_positions_, 7);
    EXPECT_EQ(num_velocities_, 6);

    num_states_ = num_positions_ + num_velocities_;

    state_vector_.resize(num_states_);
    state_vector_ << tree_->getZeroConfiguration(),
                     VectorX<double>::Zero(tree_->get_num_velocities());
    for (int i = 0; i < num_positions_; ++i) {
      std::cout << "position " << i << ": " << tree_->get_position_name(i) << std::endl;
    }
    for (int i = 0; i < num_velocities_; ++i) {
      std::cout << "velocity " << i << ": " << tree_->get_velocity_name(i) << std::endl;
    }
  }

  unique_ptr<RigidBodyTree<double>> tree_;
  unique_ptr<Magnetometer> dut_;
  unique_ptr<Context<double>> context_;
  unique_ptr<SystemOutput<double>> output_;
  const MagnetometerOutput<double>* magnetometer_output_{nullptr};
  int num_positions_{};
  int num_velocities_{};
  int num_states_{};
  VectorX<double> state_vector_;
};

// Tests that the magnetometer measures the correct value when the box is in
// various translation configurations (orientation remains the same).
TEST_F(TestMagnetometer, TestSensorTranslation) {
  // The magnetometer's reading should not be affected by the translational
  // pose of the box in the world.
  const Vector3d expected(1, 0, 0);
  for (double x = -5; x <= 5; x += 0.5) {
    for (double y = -5; y <= 5; y += 0.5) {
      for (double z = -5; z <= 5; z+= 0.5) {
        state_vector_(0) = x;  // State base_x.
        state_vector_(1) = y;  // State base_y.
        state_vector_(2) = z;  // State base_z.
        context_->FixInputPort(
            dut_->get_input_port().get_index(),
            make_unique<BasicVector<double>>(state_vector_));
        dut_->CalcOutput(*context_, output_.get());
        EXPECT_TRUE(CompareMatrices(magnetometer_output_->get_measurement(),
            expected, std::numeric_limits<double>::epsilon(),
            MatrixCompareType::absolute));
      }
    }
  }
}

// Tests that the magnetometer measures the correct value when the box is in
// various orientation configurations (translation remains the same).
TEST_F(TestMagnetometer, TestSensorOrientation) {
  // // The magnetometer's reading should not be affected by the translational
  // // pose of the box in the world.
  // const Vector3d expected(1, 0, 0);

  for (double r = -M_PI; r <= M_PI; r += M_PI_4) {
    for (double p = -M_PI; p <= M_PI; p += M_PI_4) {
      for (double y = -M_PI; y <= M_PI; y+= M_PI_4) {
        state_vector_.segment(3, 4) = rpy2quat(Vector3d(r, p, y));
        context_->FixInputPort(
            dut_->get_input_port().get_index(),
            make_unique<BasicVector<double>>(state_vector_));
        dut_->CalcOutput(*context_, output_.get());
        std::cout << "Magnetometer reading at rpy = ("
            << r << ", " << p << ", " << y << "): "
            << magnetometer_output_->get_measurement().transpose() << std::endl;
        // EXPECT_TRUE(CompareMatrices(magnetometer_output_->get_measurement(),
        //     expected, std::numeric_limits<double>::epsilon(),
        //     MatrixCompareType::absolute));
      }
    }
  }
}

// Tests that the magnetometer measurements are not affected by its velocity.
TEST_F(TestMagnetometer, TestSensorVelocity) {
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
