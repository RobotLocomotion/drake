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
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

VectorXd ComputeIiwaGravityTorque(const RigidBodyTree<double>& rigid_body_tree,
                                  const VectorXd& robot_state) {
  KinematicsCache<double> cache = rigid_body_tree.doKinematics(robot_state);
  eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                  drake::TwistVector<double>> f_ext;
  f_ext.clear();

  Eigen::VectorXd g = rigid_body_tree.dynamicsBiasTerm(cache, f_ext, false);

  // The size of this system's output vector is equal to the number of
  // actuators while the size of `g` is equal to the number of DOFs. Thus, we
  // need to extract from `g` the torque / force commands corresponding to the
  // actuators that are used.
  Eigen::VectorXd actuated_g(rigid_body_tree.get_num_actuators());
  for (int i = 0; i < rigid_body_tree.get_num_actuators(); ++i) {
    // TODO(liang.fok) The assertion below enforces that all actuators are
    // single DOF. Generalize this method to support multi-DOF actuators
    // once they exist. See #4153.
    DRAKE_ASSERT(rigid_body_tree.actuators.at(i)
                     .body_->getJoint()
                     .get_num_positions() == 1);
    int index_in_g =
        rigid_body_tree.actuators.at(i).body_->get_position_start_index();
    actuated_g[i] = g[index_in_g];
  }

  return actuated_g;
}

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class GravityCompensatorTest : public ::testing::Test {
 protected:
  GravityCompensatorTest() {}

  void SetUp() override {}

  void SetUp(std::unique_ptr<RigidBodyTree<double>> tree) {
    tree_ = std::move(tree);
    gravity_compensator_ = make_unique<GravityCompensator<double>>(*tree_);
    context_ = gravity_compensator_->CreateDefaultContext();
    output_ = gravity_compensator_->AllocateOutput(*context_);
    input_ = make_unique<BasicVector<double>>(tree_->get_num_positions());

    // Checks that the number of input ports in the Gravity Compensator system
    // and the Context are consistent.
    ASSERT_EQ(gravity_compensator_->get_num_input_ports(), 1);
    ASSERT_EQ(context_->get_num_input_ports(), 1);

    // Checks that no state variables are allocated in the context.
    EXPECT_EQ(context_->get_continuous_state()->size(), 0);

    // Checks that the number of output ports in the Gravity Compensator system
    // and the SystemOutput are consistent.
    ASSERT_EQ(output_->get_num_ports(), 1);
    ASSERT_EQ(gravity_compensator_->get_num_output_ports(), 1);
    ASSERT_NE(output_->get_vector_data(0), nullptr);
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<System<double>> gravity_compensator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
  std::unique_ptr<BasicVector<double>> input_;
};

// Tests that the expected value of the gravity compensating torque and the
// value computed by the GravityCompensator for a given joint configuration
// of the KUKA IIWA Arm are identical.
TEST_F(GravityCompensatorTest, IiwaOutputTest) {
  // The following curly brace defines a scope that quarantines local variable
  // `tree`, which is of type `std::unique_ptr`. Ownership of `tree` is passed
  // to this unit test's instance of `GravityCompensatorTest`. We quarantine
  // this local variable to prevent downstream code from seg faulting by trying
  // to access `tree` after ownership is transferred.
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        drake::multibody::joints::kFixed, nullptr /* weld to frame */,
        tree.get());
    SetUp(std::move(tree));
  }

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  input_->get_mutable_value() << robot_position;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(std::move(input_)));
  gravity_compensator_->EvalOutput(*context_, output_.get());

  VectorXd expected_gravity_vector =
      ComputeIiwaGravityTorque(*tree_, robot_position);

  // Checks the expected and computed gravity torque.
  const BasicVector<double>* output_vector = output_->get_vector_data(0);
  EXPECT_EQ(expected_gravity_vector, output_vector->get_value());
}

// Tests that the expected value of the gravity compensating torque and the
// value computed by the GravityCompensator for a given joint configuration
// of an underactuated robot are identical.
TEST_F(GravityCompensatorTest, UnderactuatedOutputTest) {
  // The following curly brace defines a scope that quarantines local variable
  // `tree`, which is of type `std::unique_ptr`. Ownership of `tree` is passed
  // to this unit test's instance of `GravityCompensatorTest`. We quarantine
  // this local variable to prevent downstream code from seg faulting by trying
  // to access `tree` after ownership is transferred.
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFile(
        drake::GetDrakePath() + "/examples/SimpleFourBar/FourBar.sdf",
        drake::multibody::joints::kFixed, nullptr /* weld to frame */,
        tree.get());
    SetUp(std::move(tree));
  }

  // Verifies that the model is indeed underactuated (it has 3 positions but
  // only one actuator).
  ASSERT_EQ(tree_->get_num_positions(), 3);
  ASSERT_EQ(tree_->get_num_actuators(), 1);

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(3);
  robot_position << 0.01, -0.02, 0.05;

  input_->get_mutable_value() << robot_position;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(std::move(input_)));
  gravity_compensator_->EvalOutput(*context_, output_.get());

  const BasicVector<double>* output_vector = output_->get_vector_data(0);

  // Checks that the output vector of the gravity compensator is of size 1
  // (there's only one actuator).
  ASSERT_EQ(output_vector->size(), 1);

  // Checks the expected and computed gravity torque.
  VectorXd expected_gravity_vector =
      ComputeIiwaGravityTorque(*tree_, robot_position);
  EXPECT_EQ(expected_gravity_vector, output_vector->get_value());
}

}  // namespace
}  // namespace systems
}  // namespace drake
