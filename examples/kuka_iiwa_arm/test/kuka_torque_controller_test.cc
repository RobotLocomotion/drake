#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using drake::systems::BasicVector;

namespace {
Eigen::VectorXd CalcGravityCompensationTorque(
    const multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& q,
    Eigen::MatrixXd* H = nullptr) {

  // Compute gravity compensation torque.
  std::unique_ptr<systems::Context<double>> plant_context =
      plant.CreateDefaultContext();
  Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(kIiwaArmNumJoints);
  plant.SetPositions(plant_context.get(), q);
  plant.SetVelocities(plant_context.get(), zero_velocity);

  if (H) {
    plant.CalcMassMatrixViaInverseDynamics(*plant_context, H);
  }

  multibody::MultibodyForces<double> external_forces(plant);
  plant.CalcForceElementsContribution(*plant_context, &external_forces);
  return plant.CalcInverseDynamics(*plant_context, zero_velocity,
                                   external_forces);
}

}  // namespace

GTEST_TEST(KukaTorqueControllerTest, GravityCompensationTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(kIiwaUrdf);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();

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
  KukaTorqueController<double> controller(plant, stiffness, damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  VectorX<double> estimated_state_input(2 * kIiwaArmNumJoints);
  estimated_state_input << q, v;

  VectorX<double> desired_state_input(2 * kIiwaArmNumJoints);
  desired_state_input << q_des, v_des;

  VectorX<double> desired_torque_input(kIiwaArmNumJoints);
  desired_torque_input << torque_des;

  controller.get_input_port_estimated_state().FixValue(context.get(),
                                                       estimated_state_input);
  controller.get_input_port_desired_state().FixValue(context.get(),
                                                     desired_state_input);
  controller.get_input_port_commanded_torque().FixValue(context.get(),
                                                        desired_torque_input);

  Eigen::VectorXd expected_torque =
      CalcGravityCompensationTorque(plant, q);

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->value(),
                              1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(KukaTorqueControllerTest, SpringTorqueTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(kIiwaUrdf);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();

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
  KukaTorqueController<double> controller(plant, stiffness, damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  VectorX<double> estimated_state_input(2 * kIiwaArmNumJoints);
  estimated_state_input << q, v;

  VectorX<double> desired_state_input(2 * kIiwaArmNumJoints);
  desired_state_input << q_des, v_des;

  VectorX<double> desired_torque_input(kIiwaArmNumJoints);
  desired_torque_input << torque_des;

  controller.get_input_port_estimated_state().FixValue(context.get(),
                                                       estimated_state_input);
  controller.get_input_port_desired_state().FixValue(context.get(),
                                                     desired_state_input);
  controller.get_input_port_commanded_torque().FixValue(context.get(),
                                                        desired_torque_input);

  // Compute gravity compensation torque.
  Eigen::VectorXd expected_torque =
      CalcGravityCompensationTorque(plant, q);

  // Compute spring torque.
  expected_torque += -((q - q_des).array() * stiffness.array()).matrix();

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->value(),
                              1e-10, MatrixCompareType::absolute));
}

GTEST_TEST(KukaTorqueControllerTest, DampingTorqueTest) {
  const std::string kIiwaUrdf =
    "manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(kIiwaUrdf);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();

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
  KukaTorqueController<double> controller(plant, stiffness, damping);

  std::unique_ptr<systems::Context<double>> context =
      controller.CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      controller.AllocateOutput();

  VectorX<double> estimated_state_input(2 * kIiwaArmNumJoints);
  estimated_state_input << q, v;

  VectorX<double> desired_state_input(2 * kIiwaArmNumJoints);
  desired_state_input << q_des, v_des;

  VectorX<double> desired_torque_input(kIiwaArmNumJoints);
  desired_torque_input << torque_des;

  controller.get_input_port_estimated_state().FixValue(context.get(),
                                                       estimated_state_input);
  controller.get_input_port_desired_state().FixValue(context.get(),
                                                     desired_state_input);
  controller.get_input_port_commanded_torque().FixValue(context.get(),
                                                        desired_torque_input);

  // Compute gravity compensation torque and mass matrix.
  Eigen::MatrixXd H(kIiwaArmNumJoints, kIiwaArmNumJoints);
  Eigen::VectorXd expected_torque =
      CalcGravityCompensationTorque(plant, q, &H);

  // Compute spring torque.
  expected_torque += -((q - q_des).array() * stiffness.array()).matrix();

  // Compute damping torque.
  Eigen::VectorXd damping_torque(7);
  for (int i = 0; i < kIiwaArmNumJoints; i++) {
    damping_torque(i) =
        -v(i) * damping(i) * 2 * std::sqrt(H(i, i) * stiffness(i));
  }
  expected_torque += damping_torque;

  // Check output.
  controller.CalcOutput(*context, output.get());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->value(),
                              1e-10, MatrixCompareType::absolute));
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
