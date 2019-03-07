#include "drake/systems/controllers/rbt_inverse_dynamics.h"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/test/rbt_compute_torque.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace controllers {
namespace rbt {
namespace {

class InverseDynamicsTest : public ::testing::Test {
 protected:
  void Init(std::unique_ptr<RigidBodyTree<double>> tree,
            const InverseDynamics<double>::InverseDynamicsMode mode) {
    rigid_body_tree_ = std::move(tree);
    inverse_dynamics_ = make_unique<InverseDynamics<double>>(
        rigid_body_tree_.get(), mode);
    FinishInit(mode);
  }

  void FinishInit(const InverseDynamics<double>::InverseDynamicsMode mode) {
    inverse_dynamics_context_ = inverse_dynamics_->CreateDefaultContext();
    output_ = inverse_dynamics_->AllocateOutput();

    // Checks that the number of input ports in the Gravity Compensator system
    // and the Context are consistent.
    if (mode == InverseDynamics<double>::kGravityCompensation) {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 1);
      EXPECT_EQ(inverse_dynamics_context_->get_num_input_ports(), 1);
    } else {
      EXPECT_EQ(inverse_dynamics_->get_num_input_ports(), 2);
      EXPECT_EQ(inverse_dynamics_context_->get_num_input_ports(), 2);
    }

    // Checks that no state variables are allocated in the context.
    EXPECT_EQ(inverse_dynamics_context_->get_continuous_state().size(), 0);

    // Checks that the number of output ports in the Gravity Compensator system
    // and the SystemOutput are consistent.
    EXPECT_EQ(output_->get_num_ports(), 1);
    EXPECT_EQ(inverse_dynamics_->get_num_output_ports(), 1);
  }

  void CheckGravityTorque(const Eigen::VectorXd& position) {
    CheckTorque(position, VectorXd::Zero(num_velocities()),
                VectorXd::Zero(num_velocities()));
  }

  void CheckTorque(const Eigen::VectorXd& position,
                   const Eigen::VectorXd& velocity,
                   const Eigen::VectorXd& acceleration_desired) {
    // desired acceleration.
    VectorXd vd_d = VectorXd::Zero(num_velocities());
    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      vd_d = acceleration_desired;
    }

    auto state_input = make_unique<BasicVector<double>>(
        num_positions() + num_velocities());
    state_input->get_mutable_value() << position, velocity;
    inverse_dynamics_context_->FixInputPort(
        inverse_dynamics_->get_input_port_estimated_state().get_index(),
        std::move(state_input));

    if (!inverse_dynamics_->is_pure_gravity_compensation()) {
      auto vd_input =
          make_unique<BasicVector<double>>(num_velocities());
      vd_input->get_mutable_value() << vd_d;
      inverse_dynamics_context_->FixInputPort(
          inverse_dynamics_->get_input_port_desired_acceleration().get_index(),
          std::move(vd_input));
    }

    // Hook input of the expected size.
    inverse_dynamics_->CalcOutput(*inverse_dynamics_context_, output_.get());

    // Compute the expected torque.
    VectorXd expected_torque = controllers_test::ComputeTorque(
        *rigid_body_tree_, position, velocity, vd_d);

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output_->get_vector_data(0);
    EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                                1e-10, MatrixCompareType::absolute));
  }

 private:
  int num_positions() const {
    return rigid_body_tree_->get_num_positions();
  }

  int num_velocities() const {
    return rigid_body_tree_->get_num_velocities();
  }

  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
  std::unique_ptr<InverseDynamics<double>> inverse_dynamics_;
  std::unique_ptr<Context<double>> inverse_dynamics_context_;
  std::unique_ptr<SystemOutput<double>> output_;
};

// Tests that the expected value of the gravity compensating torque and the
// value computed by the InverseDynamics in pure gravity compensation mode
// for a given joint configuration of the KUKA IIWA Arm are identical.
TEST_F(InverseDynamicsTest, GravityCompensationTestRBT) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::FindResourceOrThrow("drake/manipulation/models/"
          "iiwa_description/urdf/iiwa14_primitive_collision.urdf"),
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree),
       InverseDynamics<double>::InverseDynamicsMode::kGravityCompensation);

  // Defines an arbitrary robot position vector.
  Eigen::VectorXd robot_position = Eigen::VectorXd::Zero(7);
  robot_position << 0.01, -0.01, 0.01, 0.5, 0.01, -0.01, 0.01;

  CheckGravityTorque(robot_position);
}

// Tests that inverse dynamics returns the expected torque for a given state and
// desired acceleration for the iiwa arm.
TEST_F(InverseDynamicsTest, InverseDynamicsTestRBT) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      drake::FindResourceOrThrow("drake/manipulation/models/"
      "iiwa_description/urdf/iiwa14_primitive_collision.urdf"),
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());
  Init(std::move(tree),
       InverseDynamics<double>::InverseDynamicsMode::kInverseDynamics);

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

}  // namespace
}  // namespace rbt
}  // namespace controllers
}  // namespace systems
}  // namespace drake
