#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/controllers/test_utilities/compute_torque.h"

using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {
namespace {

class InverseDynamicsControllerTest : public ::testing::Test {
 protected:
  void SetPidGains(VectorX<double>* kp, VectorX<double>* ki,
                   VectorX<double>* kd) {
    *kp << 1, 2, 3, 4, 5, 6, 7;
    *ki << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    *kd = *kp / 2.;
  }

  void ConfigTestAndCheck(InverseDynamicsController<double>* test_sys,
                          const VectorX<double>& kp, const VectorX<double>& ki,
                          const VectorX<double>& kd) {
    auto inverse_dynamics_context = test_sys->CreateDefaultContext();
    auto output = test_sys->AllocateOutput();
    const MultibodyPlant<double>& robot_plant =
        *test_sys->get_multibody_plant_for_control();

    // Sets current state and reference state and acceleration values.
    const int dim = robot_plant.num_positions();
    VectorX<double> q(dim), v(dim), q_r(dim), v_r(dim), vd_r(dim);
    q << 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3;
    v = q * 3;

    q_r = (q + VectorX<double>::Constant(dim, 0.1)) * 2.;
    v_r.setZero();
    vd_r << 1, 2, 3, 4, 5, 6, 7;

    // Connects inputs.
    VectorX<double> state_input(robot_plant.num_positions() +
                                robot_plant.num_velocities());
    state_input << q, v;

    VectorX<double> reference_state_input(robot_plant.num_positions() +
                                          robot_plant.num_velocities());
    reference_state_input << q_r, v_r;

    VectorX<double> reference_acceleration_input(robot_plant.num_velocities());
    reference_acceleration_input << vd_r;

    test_sys->get_input_port_estimated_state().FixValue(
        inverse_dynamics_context.get(), state_input);
    test_sys->get_input_port_desired_state().FixValue(
        inverse_dynamics_context.get(), reference_state_input);
    test_sys->get_input_port_desired_acceleration().FixValue(
        inverse_dynamics_context.get(), reference_acceleration_input);

    // Sets integrated position error.
    VectorX<double> q_int(dim);
    q_int << -1, -2, -3, -4, -5, -6, -7;
    test_sys->set_integral_value(inverse_dynamics_context.get(), q_int);

    // Computes output.
    test_sys->CalcOutput(*inverse_dynamics_context, output.get());

    // The results should equal to this.
    VectorX<double> vd_d = (kp.array() * (q_r - q).array()).matrix() +
                           (kd.array() * (v_r - v).array()).matrix() +
                           (ki.array() * q_int.array()).matrix() + vd_r;

    auto robot_context = robot_plant.CreateDefaultContext();
    VectorX<double> expected_torque = controllers_test::ComputeTorque(
        robot_plant, q, v, vd_d, robot_context.get());

    // Checks the expected and computed gravity torque.
    const BasicVector<double>* output_vector = output->get_vector_data(0);
    EXPECT_TRUE(CompareMatrices(expected_torque, output_vector->get_value(),
                                1e-10, MatrixCompareType::absolute));
  }
};

// Tests the computed torque from InverseDynamicsController matches hand
// derived results for the kuka iiwa arm at a given state (q, v), when
// asked to track reference state (q_r, v_r) and reference acceleration (vd_r).
// This test verifies the case that inverse dynamics controller only references
// the input robot plant.
TEST_F(InverseDynamicsControllerTest, TestTorqueWithReferencedPlant) {
  auto robot = std::make_unique<MultibodyPlant<double>>(0.0);
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  multibody::Parser(robot.get()).AddModelFromFile(full_name);
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();

  // Sets pid gains.
  const int dim = robot->num_positions();
  VectorX<double> kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto dut = std::make_unique<InverseDynamicsController<double>>(
      *robot, kp, ki, kd, true /* expose reference acceleration port */);

  ConfigTestAndCheck(dut.get(), kp, ki, kd);
}

// Tests the computed torque. This test is similar to the previous test. The
// difference is that the inverse dynamics controller is created by owning the
// input robot plant.
TEST_F(InverseDynamicsControllerTest, TestTorqueWithOwnedPlant) {
  auto robot = std::make_unique<MultibodyPlant<double>>(0.0);
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  multibody::Parser(robot.get()).AddModelFromFile(full_name);
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();

  // Sets pid gains.
  const int dim = robot->num_positions();
  VectorX<double> kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto dut = std::make_unique<InverseDynamicsController<double>>(
      std::move(robot), kp, ki, kd,
      true /* expose reference acceleration port */);

  ConfigTestAndCheck(dut.get(), kp, ki, kd);
}


}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
