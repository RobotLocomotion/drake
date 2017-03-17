#include "drake/systems/controllers/inverse_dynamics.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

VectorXd ComputeTorque(const RigidBodyTree<double>& tree, const VectorXd& q,
                       const VectorXd& v, const VectorXd& vd_d) {
  // Compute the expected torque.
  KinematicsCache<double> cache = tree.doKinematics(q, v);
  eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                  drake::TwistVector<double>>
      f_ext;

  return tree.massMatrix(cache) * vd_d + tree.dynamicsBiasTerm(cache, f_ext);
}

class InverseDynamicsTest : public ::testing::Test {
 protected:
  void Init(std::unique_ptr<RigidBodyTree<double>> tree,
            bool pure_gravity_compensation) {
    tree_ = std::move(tree);
    inverse_dynamics_ = make_unique<InverseDynamics<double>>(
        *tree_, pure_gravity_compensation /* pure gravity compensation mode */);
    context_ = inverse_dynamics_->CreateDefaultContext();
    output_ = inverse_dynamics_->AllocateOutput(*context_);

    // Checks that the number of input ports in the Gravity Compensator system
    // and the Context are consistent.
    if (pure_gravity_compensation) {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 1);
      EXPECT_EQ(context_->get_num_input_ports(), 1);
    } else {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 2);
      EXPECT_EQ(context_->get_num_input_ports(), 2);
    }

    // Checks that no state variables are allocated in the context.
    EXPECT_EQ(context_->get_continuous_state()->size(), 0);

    // Checks that the number of output ports in the Gravity Compensator system
    // and the SystemOutput are consistent.
    EXPECT_EQ(output_->get_num_ports(), 1);
    EXPECT_EQ(inverse_dynamics_->get_num_output_ports(), 1);
  }

  void CheckGravityTorque(const Eigen::VectorXd& position) {
    CheckTorque(position, VectorXd::Zero(tree_->get_num_velocities()),
                VectorXd::Zero(tree_->get_num_velocities()));
  }

  void CheckTorque(const Eigen::VectorXd& position,
                   const Eigen::VectorXd& velocity,
                   const Eigen::VectorXd& acceleration_desired) {
    // desired acceleration.
    VectorXd vd_d = VectorXd::Zero(tree_->get_num_velocities());
    if (!inverse_dynamics_->is_pure_gravity_compenstation()) {
      vd_d = acceleration_desired;
    }

    auto state_input = make_unique<BasicVector<double>>(
        tree_->get_num_positions() + tree_->get_num_velocities());
    state_input->get_mutable_value() << position, velocity;
    context_->FixInputPort(
        inverse_dynamics_->get_input_port_estimated_state().get_index(),
        std::move(state_input));

    if (!inverse_dynamics_->is_pure_gravity_compenstation()) {
      auto vd_input =
          make_unique<BasicVector<double>>(tree_->get_num_velocities());
      vd_input->get_mutable_value() << vd_d;
      context_->FixInputPort(
          inverse_dynamics_->get_input_port_desired_acceleration().get_index(),
          std::move(vd_input));
    }

    // Hook input of the expected size.
    inverse_dynamics_->CalcOutput(*context_, output_.get());

    // Compute the expected torque.
    VectorXd expected_torque = ComputeTorque(*tree_, position, velocity, vd_d);

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                                1e-10, MatrixCompareType::absolute));
  }

  std::unique_ptr<RigidBodyTree<double>> tree_;
  std::unique_ptr<InverseDynamics<double>> inverse_dynamics_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the expected value of the gravity compensating torque and the
// value computed by the InverseDynamics in pure gravity compensation mode
// for a given joint configuration of the KUKA IIWA Arm are identical.
TEST_F(InverseDynamicsTest, GravityCompensationTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree), true /* pure gravity compensation */);

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  CheckGravityTorque(robot_position);
}

// Tests that inverse dynamics returns the expected torque for a given state and
// desired acceleration for the iiwa arm.
TEST_F(InverseDynamicsTest, InverseDynamicsTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree), false /* inverse dynamics */);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd vd_d = Eigen::VectorXd::Zero(7);
  for (int i = 0; i < 7; ++i) {
    q[i] = i * 0.1 - 0.3;
    v[i] = i - 3;
    vd_d[i] = i - 3;
  }

  CheckTorque(q, v, vd_d);
}

// Tests that the GravityCompensator will abort if it is provided an
// underactuated model.
TEST_F(InverseDynamicsTest, UnderactuatedModelTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::sdf::AddModelInstancesFromSdfFile(
      drake::GetDrakePath() + "/examples/SimpleFourBar/FourBar.sdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  EXPECT_DEATH(Init(std::move(tree), true), ".*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
