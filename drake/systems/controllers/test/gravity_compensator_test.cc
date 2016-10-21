#include "drake/systems/controllers/gravity_compensator.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

VectorXd ComputeIiwaGravityTorque(const VectorXd& robot_state) {
  RigidBodyTree rigid_body_tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  KinematicsCache<double> cache = rigid_body_tree.doKinematics(robot_state);
  eigen_aligned_std_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();

  return rigid_body_tree.dynamicsBiasTerm(cache, f_ext, false);
}

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class GravityCompensatorTest : public ::testing::Test {
 protected:
  GravityCompensatorTest() {
    tree_ = std::make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        drake::systems::plants::joints::kFixed, nullptr /* weld to frame */,
        tree_.get());
  }

  void SetUp() override {
    gravity_compensator_ = make_unique<GravityCompensator<double>>(*tree_);
    context_ = gravity_compensator_->CreateDefaultContext();
    output_ = gravity_compensator_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(7 /* length */);
  }

  std::unique_ptr<RigidBodyTree> tree_;
  std::unique_ptr<System<double>> gravity_compensator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};


// Tests that the expected value of the gravity compensating torque and the
// value computed by the GravityCompensator for a given joint configuration
// on the IIWA Arm are identical.
TEST_F(GravityCompensatorTest, OutputTest) {
  // Checks that the number of input ports in the Gravity Compensator system
  // and the Context are consistent.
  ASSERT_EQ(1, gravity_compensator_->get_num_input_ports());
  ASSERT_EQ(1, context_->get_num_input_ports());

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  input_->get_mutable_value() << robot_position;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(std::move(input_)));
  gravity_compensator_->EvalOutput(*context_, output_.get());

  // Checks that the number of output ports in the Gravity Compensator system
  // and the SystemOutput are consistent.
  ASSERT_EQ(1, output_->get_num_ports());
  ASSERT_EQ(1, gravity_compensator_->get_num_output_ports());
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);

  VectorXd expected_gravity_vector = ComputeIiwaGravityTorque(robot_position);

  // Checks the expected and computed gravity torque.
  EXPECT_EQ(expected_gravity_vector, output_vector->get_value());
}

// Tests that Gain allocates no state variables in the context.
TEST_F(GravityCompensatorTest, GravityCompensatorIsStateless) {
  EXPECT_EQ(0, context_->get_continuous_state()->size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
