#include "drake/systems/controllers/gravity_compensator.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"



using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

VectorXd ComputeGravityTorque(const RigidBodyTree<double>& rigid_body_tree,
                                  const VectorXd& robot_state) {
  KinematicsCache<double> cache = rigid_body_tree.doKinematics(robot_state);
  eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                  drake::TwistVector<double>> f_ext;
  f_ext.clear();

  return rigid_body_tree.dynamicsBiasTerm(cache, f_ext, false);
}

class GravityCompensatorTest : public ::testing::Test {
 protected:
  void Init(std::unique_ptr<RigidBodyTree<double>> tree) {
    tree_ = std::move(tree);
    gravity_compensator_ = make_unique<GravityCompensator<double>>(*tree_);
    context_ = gravity_compensator_->CreateDefaultContext();
    output_ = gravity_compensator_->AllocateOutput(*context_);

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

  void CheckConfiguration(const Eigen::VectorXd& position_vector) {
    EXPECT_EQ(position_vector.size(), tree_->get_num_positions());
    auto input = make_unique<BasicVector<double>>(tree_->get_num_positions());
    input->get_mutable_value() << position_vector;

    // Hook input of the expected size.
    context_->FixInputPort(0, std::move(input));
    gravity_compensator_->CalcOutput(*context_, output_.get());

    VectorXd expected_gravity_vector =
      ComputeGravityTorque(*tree_, position_vector);

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    EXPECT_EQ(expected_gravity_vector, output_vector->get_value());
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<System<double>> gravity_compensator_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the expected value of the gravity compensating torque and the
// value computed by the GravityCompensator for a given joint configuration
// of the KUKA IIWA Arm are identical.
TEST_F(GravityCompensatorTest, IiwaOutputTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree));

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  CheckConfiguration(robot_position);
}

// Tests that the GravityCompensator will abort if it is provided an
// underactuated model.
TEST_F(GravityCompensatorTest, UnderactuatedModelTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/examples/SimpleFourBar/FourBar.sdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  EXPECT_DEATH(Init(std::move(tree)), ".*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
