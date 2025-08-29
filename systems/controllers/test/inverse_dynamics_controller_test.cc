#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/controllers/test_utilities/compute_torque.h"

using drake::multibody::MultibodyPlant;
using Eigen::VectorXd;

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
                          const VectorX<double>& kd,
                          const Context<double>* robot_context = nullptr) {
    auto inverse_dynamics_context = test_sys->CreateDefaultContext();
    const MultibodyPlant<double>& robot_plant =
        *test_sys->get_multibody_plant_for_control();

    // Sets current state and reference state and acceleration values.
    const int dim = robot_plant.num_positions();
    VectorX<double> q(dim), v(dim), q_r(dim), v_r(dim), vd_r(dim);
    q = Eigen::VectorXd::LinSpaced(dim, 0.3, -0.3);
    v = q * 3;

    q_r = (q + VectorX<double>::Constant(dim, 0.1)) * 2.;
    v_r.setZero();
    vd_r = Eigen::VectorXd::LinSpaced(dim, 1, dim);

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
    VectorX<double> q_int = Eigen::VectorXd::LinSpaced(dim, -1, -7);
    test_sys->set_integral_value(inverse_dynamics_context.get(), q_int);

    // The results should equal to this.
    VectorX<double> vd_d = (kp.array() * (q_r - q).array()).matrix() +
                           (kd.array() * (v_r - v).array()).matrix() +
                           (ki.array() * q_int.array()).matrix() + vd_r;

    std::unique_ptr<Context<double>> workspace_context =
        robot_context == nullptr ? robot_plant.CreateDefaultContext()
                                 : robot_context->Clone();
    VectorX<double> expected_generalized_force =
        controllers_test::ComputeTorque(robot_plant, q, v, vd_d,
                                        workspace_context.get());
    Eigen::VectorXd expected_actuation =
        robot_plant.MakeActuationMatrixPseudoinverse() *
        expected_generalized_force;

    // Checks the expected and computed gravity torque.
    const Eigen::VectorXd& actuation =
        test_sys->get_output_port_control().Eval(*inverse_dynamics_context);
    EXPECT_TRUE(CompareMatrices(expected_actuation, actuation, 1e-10,
                                MatrixCompareType::absolute));

    const Eigen::VectorXd& generalized_force =
        test_sys->get_output_port_generalized_force().Eval(
            *inverse_dynamics_context);
    EXPECT_TRUE(CompareMatrices(expected_generalized_force, generalized_force,
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
  multibody::Parser(robot.get())
      .AddModelsFromUrl(
          "package://drake_models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf");
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();

  // Sets pid gains.
  const int dim = robot->num_positions();
  VectorX<double> kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto dut = std::make_unique<InverseDynamicsController<double>>(
      *robot, kp, ki, kd, true /* expose reference acceleration port */);
  ConfigTestAndCheck(dut.get(), kp, ki, kd);

  // Test with custom context.
  auto custom_context = robot->CreateDefaultContext();
  const auto& iiwa_link_7 = robot->GetBodyByName("iiwa_link_7");
  iiwa_link_7.SetMass(custom_context.get(), 10.0);
  dut = std::make_unique<InverseDynamicsController<double>>(
      *robot, kp, ki, kd, true, custom_context.get());
  ConfigTestAndCheck(dut.get(), kp, ki, kd, custom_context.get());
}

// Tests the computed torque. This test is similar to the previous test. The
// difference is that the inverse dynamics controller is created by owning the
// input robot plant.
TEST_F(InverseDynamicsControllerTest, TestTorqueWithOwnedPlant) {
  auto robot = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(robot.get())
      .AddModelsFromUrl(
          "package://drake_models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf");
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

// Tests the computed torque. This test is similar to the previous test. The
// difference is that a custom robot context is used.
TEST_F(InverseDynamicsControllerTest,
       TestTorqueWithOwnedPlantAndCustomContext) {
  auto robot = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(robot.get())
      .AddModelsFromUrl(
          "package://drake_models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf");
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();

  // Sets pid gains.
  const int dim = robot->num_positions();
  VectorX<double> kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  // Create custom context.
  auto custom_context = robot->CreateDefaultContext();
  const auto& iiwa_link_7 = robot->GetBodyByName("iiwa_link_7");
  iiwa_link_7.SetMass(custom_context.get(), 10.0);

  auto dut = std::make_unique<InverseDynamicsController<double>>(
      std::move(robot), kp, ki, kd, true, custom_context.get());

  ConfigTestAndCheck(dut.get(), kp, ki, kd, custom_context.get());
}

// Tests the case when the actuators are ordered differently than the
// generalized forces (so B is not the identity matrix).
TEST_F(InverseDynamicsControllerTest, ActuatorOrder) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body name="upper_arm" pos="0 0 2">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom name="upper_arm" fromto="0 0 0 0 0 1" size="0.05" type="capsule" mass="1"/>
      <body name="lower_arm" pos="0 0 1">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom name="lower_arm" fromto="0 0 0 0 0 1" size="0.049" type="capsule" mass="1"/>
      </body>
    </body>
  </worldbody>
   <actuator>
    <!-- intentionally list these in reverse order -->
    <motor name="elbow" joint="elbow"/>
    <motor name="shoulder" joint="shoulder"/>
  </actuator>
</mujoco>
)""";
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromString(xml, ".xml");
  mbp->Finalize();
  EXPECT_EQ(mbp->num_positions(), 2);
  EXPECT_EQ(mbp->num_velocities(), 2);
  EXPECT_EQ(mbp->num_actuators(), 2);
  EXPECT_FALSE(mbp->MakeActuationMatrix().isIdentity());

  const int dim = mbp->num_positions();
  VectorX<double> kp(dim), ki(dim), kd(dim);
  kp << 1, 2;
  ki << 0.1, 0.2;
  kd = kp / 2.;

  auto dut = std::make_unique<InverseDynamicsController<double>>(
      std::move(mbp), kp, ki, kd, true);

  ConfigTestAndCheck(dut.get(), kp, ki, kd);
}

GTEST_TEST(AdditionalInverseDynamicsTest, ScalarConversion) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  multibody::Parser(mbp.get()).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));
  mbp->Finalize();

  const int num_states = mbp->num_multibody_states();
  const int dim = mbp->num_positions();
  VectorXd kp = VectorXd::Constant(dim, 0.12),
           ki = VectorXd::Constant(dim, 0.34),
           kd = VectorXd::Constant(dim, 0.56);

  InverseDynamicsController<double> idc(*mbp, kp, ki, kd, true);

  // Test AutoDiffXd.
  auto idc_ad = System<double>::ToAutoDiffXd<Diagram>(idc);
  // Note: With the current scalar conversion support, we can get a
  // unique_ptr<Diagram<T>> but not a unique_ptr<InverseDynamicsController<T>>.

  // Check the multibody plant.
  EXPECT_EQ(idc_ad->get_input_port(0).size(), num_states);
  // Check the PID gains.
  const auto* pid_ad = dynamic_cast<const PidController<AutoDiffXd>*>(
      &idc_ad->GetSubsystemByName("pid"));
  ASSERT_NE(pid_ad, nullptr);
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Kp_vector(), kp));
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Ki_vector(), ki));
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Kd_vector(), kd));
  // Check has_reference_acceleration.
  EXPECT_EQ(idc_ad->num_input_ports(), 3);

  // Test Expression.
  auto idc_sym = idc.ToSymbolic();
  EXPECT_EQ(idc_sym->get_input_port(0).size(), num_states);

  InverseDynamicsController<double> idc_with_ownership(std::move(mbp), kp, ki,
                                                       kd, true);

  // Test AutoDiffXd.
  idc_ad = System<double>::ToAutoDiffXd<Diagram>(idc_with_ownership);
  // Note: With the current scalar conversion support, we can get a
  // unique_ptr<Diagram<T>> but not a unique_ptr<InverseDynamicsController<T>>.

  // Check the multibody plant.
  EXPECT_EQ(idc_ad->get_input_port(0).size(), num_states);
  // Check the PID gains.
  pid_ad = dynamic_cast<const PidController<AutoDiffXd>*>(
      &idc_ad->GetSubsystemByName("pid"));
  ASSERT_NE(pid_ad, nullptr);
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Kp_vector(), kp));
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Ki_vector(), ki));
  EXPECT_TRUE(CompareMatrices(pid_ad->get_Kd_vector(), kd));
  // Check has_reference_acceleration.
  EXPECT_EQ(idc_ad->num_input_ports(), 3);

  // Test Expression.
  idc_sym = idc_with_ownership.ToSymbolic();
  EXPECT_EQ(idc_sym->get_input_port(0).size(), num_states);
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
