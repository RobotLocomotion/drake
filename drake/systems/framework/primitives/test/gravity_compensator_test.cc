#include "drake/systems/framework/primitives/gravity_compensator.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"

#include "drake/systems/plants/parser_urdf.h"

#include "drake/systems/plants/RigidBodyTree.h"
#include <memory>
#include <stdexcept>
#include <string>

#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

VectorXd ComputeIiwaGravityTorque(VectorXd robot_state)
{
  RigidBodyTree rigid_body_tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);
  int num_DoF = rigid_body_tree.number_of_positions();
  KinematicsCache<double> cache =
      rigid_body_tree.doKinematics(
          robot_state.head(num_DoF), robot_state.tail(num_DoF));
  eigen_aligned_unordered_map<RigidBody const*, drake::TwistVector<double>>
      f_ext;
  f_ext.clear();
  Eigen::VectorXd vd(num_DoF);
  vd.setZero();

  Eigen::VectorXd G = rigid_body_tree.inverseDynamics(cache, f_ext, vd,
                                                       false);
  return(G);
}

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class GravityCompensatorTest : public ::testing::Test {
 protected:

  void SetUp() override {
    tree_ = make_unique<RigidBodyTree>();
//    tree_->
//    (RigidBodyTree(drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
//                        DrakeJoint::FIXED)) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
    drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
    DrakeJoint::FIXED, nullptr /* weld to frame */, tree_.get());

    gravity_compensator_ = make_unique<GravityCompensator<double>>(*move(tree_).get());
    context_ = gravity_compensator_->CreateDefaultContext();
    output_ = gravity_compensator_->AllocateOutput(*context_);
    input0_ = make_unique<BasicVector<double>>(7 /* length */);
    input1_ = make_unique<BasicVector<double>>(7 /* length */);
  }

  std::unique_ptr<RigidBodyTree> tree_;
  std::unique_ptr<System<double>> gravity_compensator_;
  std::unique_ptr<ContextBase<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input0_;
  std::unique_ptr<BasicVector<double>> input1_;
};

TEST_F(GravityCompensatorTest, VectorThroughGainSystem) {
// Checks that the number of input ports in the Gravity Compensator system and the
// Context are consistent.
ASSERT_EQ(1, gravity_compensator_->get_num_input_ports());
ASSERT_EQ(1, context_->get_num_input_ports());

// robot state vector (positions and velocities)
Eigen::VectorXd robot_state = Eigen::VectorXd::Zero(14,1);

input0_->get_mutable_value() << robot_state;

// Hook input of the expected size.
context_->SetInputPort(0, MakeInput(std::move(input0_)));

gravity_compensator_->EvalOutput(*context_, output_.get());

// Checks that the number of output ports in the Gravity Compensator system and
// the SystemOutput are consistent.
ASSERT_EQ(1, output_->get_num_ports());
ASSERT_EQ(1, gravity_compensator_->get_num_output_ports());
const BasicVector<double>* output_vector =
    dynamic_cast<const BasicVector<double>*>(output_->get_vector_data(0));
ASSERT_NE(nullptr, output_vector);

VectorXd expected_gravity_vector = ComputeIiwaGravityTorque(robot_state);

EXPECT_EQ(expected_gravity_vector, output_vector->get_value());
}

// Tests that Gain allocates no state variables in the context_.
TEST_F(GravityCompensatorTest, GravityCompensatorIsStateless) {
EXPECT_EQ(nullptr, context_->get_state().continuous_state);
}

}  // namespace
}  // namespace systems
}  // namespace drake
