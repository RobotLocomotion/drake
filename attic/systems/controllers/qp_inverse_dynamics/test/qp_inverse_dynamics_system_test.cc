#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_system.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"
#include "drake/systems/controllers/setpoint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {
namespace {

// Tests that the QpInverseDynamicsSystem solves a simple inverse dynamics
// problem for the iiwa arm. Without any external contact, hitting any torque
// constraints, the inverse dynamics controller should track the desired
// acceleration almost perfectly.
GTEST_TEST(testQpInverseDynamicsSystem, IiwaInverseDynamics) {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");

  const std::string kAliasGroupsPath = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "iiwa.alias_groups");

  const std::string kControlConfigPath = FindResourceOrThrow(
      "drake/systems/controllers/qp_inverse_dynamics/test/"
      "iiwa.id_controller_config");

  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelPath, multibody::joints::kFixed, &robot);

  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(kAliasGroupsPath);
  ParamSet paramset;
  paramset.LoadFromFile(kControlConfigPath, alias_groups);

  DiagramBuilder<double> builder;

  // Makes a controller block.
  const double kControlDt = 0.02;
  QpInverseDynamicsSystem* controller =
      builder.AddSystem<QpInverseDynamicsSystem>(&robot, kControlDt);
  controller->set_name("controller");

  // Makes a source for humanoid status.
  RobotKinematicState<double> robot_status(&robot);
  const VectorX<double> q = VectorX<double>::Zero(robot.get_num_positions());
  const VectorX<double> v = VectorX<double>::Zero(robot.get_num_velocities());

  robot_status.UpdateKinematics(0 /* time */, q, v);
  ConstantValueSource<double>* state_source =
      builder.AddSystem<ConstantValueSource<double>>(
          Value<RobotKinematicState<double>>(robot_status));
  state_source->set_name("state_source");

  // Makes a source for qp input.
  const std::vector<std::string> empty;
  QpInput input = paramset.MakeQpInput(empty, /* contacts */
                                       empty, /* tracked bodies*/
                                       alias_groups);
  VectorX<double> kp, kd;
  paramset.LookupDesiredDofMotionGains(&kp, &kd);
  // Desired q and v.
  VectorX<double> q_d = q;
  VectorX<double> v_d = v;
  q_d[2] += 1;
  v_d[5] -= 0.5;
  const VectorSetpoint<double> policy(
      q_d, v_d, VectorX<double>::Zero(v_d.size()), kp, kd);
  // Uses policy to generate desired acceleration.
  input.mutable_desired_dof_motions().mutable_values() =
      policy.ComputeTargetAcceleration(q, v);

  ConstantValueSource<double>* qp_input_source =
      builder.AddSystem<ConstantValueSource<double>>(
          Value<QpInput>(input));
  qp_input_source->set_name("qp_input_source");

  // Connects the diagram.
  builder.Connect(state_source->get_output_port(0),
                  controller->get_input_port_kinematic_state());

  builder.Connect(qp_input_source->get_output_port(0),
                  controller->get_input_port_qp_input());

  builder.ExportOutput(controller->get_output_port_qp_output());

  auto diagram = builder.Build();

  // Uses the simulator to avoid various allocations by hand.
  Simulator<double> sim(*diagram);
  std::unique_ptr<SystemOutput<double>> output =
      diagram->AllocateOutput();
  sim.Initialize();
  sim.AdvanceTo(controller->get_control_dt());

  // Gets the output from the plan eval block.
  diagram->CalcOutput(sim.get_context(), output.get());
  const QpOutput& qp_output = output->get_data(0)->get_value<QpOutput>();

  // Expects the solved acceleration to match the desired acceleration well.
  EXPECT_TRUE(drake::CompareMatrices(input.desired_dof_motions().values(),
                                     qp_output.vd(), 1e-9,
                                     drake::MatrixCompareType::absolute));

  // Without any external forces or hitting any constraints, torque = M * vd_d +
  // h.
  VectorX<double> expected_torque =
      robot_status.get_M() * input.desired_dof_motions().values() +
      robot_status.get_bias_term();
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, qp_output.dof_torques(),
                                     1e-9, drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
