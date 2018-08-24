#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using drake::systems::BasicVector;

GTEST_TEST(KukaTorqueControllerTest, GravityCompensationTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kIiwaUrdf, multibody::joints::kFixed, tree_ptr.get());
  const RigidBodyTree<double>& tree = *tree_ptr;

  // Set stiffness and damping to zero.
  VectorX<double> stiffness(kIiwaArmNumJoints);
  stiffness.setZero();
  VectorX<double> damping(kIiwaArmNumJoints);
  damping.setZero();

  // Choose an arbitrary state.
  VectorX<double> q(7);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorX<double> v(7);
  v << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  VectorX<double> q_des(7);
  q_des << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  VectorX<double> v_des(7);
  v_des << -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1;
  VectorX<double> torque_des(kIiwaArmNumJoints);
  torque_des.setZero();

  // Compute controller output.
  KukaTorqueController<double> controller(std::move(tree_ptr), stiffness,
                                          damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  auto estimated_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  estimated_state_input->get_mutable_value() << q, v;

  auto desired_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  desired_state_input->get_mutable_value() << q_des, v_des;

  auto desired_torque_input =
      std::make_unique<BasicVector<double>>(kIiwaArmNumJoints);
  desired_torque_input->get_mutable_value() << torque_des;

  context->FixInputPort(
      controller.get_input_port_estimated_state().get_index(),
      std::move(estimated_state_input));
  context->FixInputPort(controller.get_input_port_desired_state().get_index(),
                        std::move(desired_state_input));
  context->FixInputPort(
      controller.get_input_port_commanded_torque().get_index(),
      std::move(desired_torque_input));

  // Compute gravity compensation torque.
  Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  auto cache = tree.doKinematics(q, zero_velocity);
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  auto expected_torque =
      tree.dynamicsBiasTerm(cache, no_external_wrenches, false);

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                              1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(KukaTorqueControllerTest, SpringTorqueTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kIiwaUrdf, multibody::joints::kFixed, tree_ptr.get());
  const RigidBodyTree<double>& tree = *tree_ptr;

  // Set nonzero stiffness and zero damping.
  VectorX<double> stiffness(kIiwaArmNumJoints);
  stiffness << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorX<double> damping(kIiwaArmNumJoints);
  damping.setZero();

  // Choose an arbitrary state
  VectorX<double> q(7);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorX<double> v(7);
  v << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  VectorX<double> q_des(7);
  q_des << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  VectorX<double> v_des(7);
  v_des << -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1;
  VectorX<double> torque_des(kIiwaArmNumJoints);
  torque_des.setZero();

  // Compute controller output.
  KukaTorqueController<double> controller(std::move(tree_ptr), stiffness,
                                          damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  auto estimated_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  estimated_state_input->get_mutable_value() << q, v;

  auto desired_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  desired_state_input->get_mutable_value() << q_des, v_des;

  auto desired_torque_input =
      std::make_unique<BasicVector<double>>(kIiwaArmNumJoints);
  desired_torque_input->get_mutable_value() << torque_des;

  context->FixInputPort(
      controller.get_input_port_estimated_state().get_index(),
      std::move(estimated_state_input));
  context->FixInputPort(controller.get_input_port_desired_state().get_index(),
                        std::move(desired_state_input));
  context->FixInputPort(
      controller.get_input_port_commanded_torque().get_index(),
      std::move(desired_torque_input));

  // Compute gravity compensation torque.
  Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  auto cache = tree.doKinematics(q, zero_velocity);
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  auto expected_torque =
      tree.dynamicsBiasTerm(cache, no_external_wrenches, false);

  // Compute spring torque.
  expected_torque += -((q - q_des).array() * stiffness.array()).matrix();

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                              1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(KukaTorqueControllerTest, DampingTorqueTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kIiwaUrdf, multibody::joints::kFixed, tree_ptr.get());
  const RigidBodyTree<double>& tree = *tree_ptr;

  // Set arbitrary stiffness and damping.
  VectorX<double> stiffness(kIiwaArmNumJoints);
  stiffness << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorX<double> damping(kIiwaArmNumJoints);
  damping << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;

  // Choose an arbitrary state
  VectorX<double> q(7);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VectorX<double> v(7);
  v << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  VectorX<double> q_des(7);
  q_des << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  VectorX<double> v_des(7);
  v_des << -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1;
  VectorX<double> torque_des(kIiwaArmNumJoints);
  torque_des.setZero();

  // Compute controller output.
  KukaTorqueController<double> controller(std::move(tree_ptr), stiffness,
                                          damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  auto estimated_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  estimated_state_input->get_mutable_value() << q, v;

  auto desired_state_input =
      std::make_unique<BasicVector<double>>(2 * kIiwaArmNumJoints);
  desired_state_input->get_mutable_value() << q_des, v_des;

  auto desired_torque_input =
      std::make_unique<BasicVector<double>>(kIiwaArmNumJoints);
  desired_torque_input->get_mutable_value() << torque_des;

  context->FixInputPort(
      controller.get_input_port_estimated_state().get_index(),
      std::move(estimated_state_input));
  context->FixInputPort(controller.get_input_port_desired_state().get_index(),
                        std::move(desired_state_input));
  context->FixInputPort(
      controller.get_input_port_commanded_torque().get_index(),
      std::move(desired_torque_input));

  // Compute gravity compensation torque.
  Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  auto cache = tree.doKinematics(q, zero_velocity);
  const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  auto expected_torque =
      tree.dynamicsBiasTerm(cache, no_external_wrenches, false);

  // Compute spring torque.
  expected_torque += -((q - q_des).array() * stiffness.array()).matrix();

  // Compute damping torque.
  Eigen::VectorXd damping_torque(7);
  auto M = tree.massMatrix(cache);
  for (int i = 0; i < kIiwaArmNumJoints; i++) {
    damping_torque(i) =
        -v(i) * damping(i) * 2 * std::sqrt(M(i, i) * stiffness(i));
  }
  expected_torque += damping_torque;

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
