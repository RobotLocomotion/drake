#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// Tests that the QpControllerSystem solves a simple inverse dynamics problem
// for the iiwa arm. Without any external contact, hitting any torque
// constraints, the inverse dynamics controller should track the desired
// acceleration almost perfectly.
GTEST_TEST(testQpControllerSystem, IiwaInverseDynamics) {
  const std::string kModelPath =
      drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";

  const std::string kAliasGroupsPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/config/"
      "iiwa.alias_groups";

  const std::string kControlConfigPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/config/"
      "iiwa.id_controller_config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelPath, multibody::joints::kFixed, robot.get());

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
  alias_groups.LoadFromFile(kAliasGroupsPath);
  param_parsers::ParamSet paramset;
  paramset.LoadFromFile(kControlConfigPath, alias_groups);

  systems::DiagramBuilder<double> builder;

  // Makes a controller block.
  const double kControlDt = 0.02;
  QpControllerSystem* controller =
      builder.AddSystem<QpControllerSystem>(*robot, kControlDt);

  // Makes a source for humanoid status.
  HumanoidStatus robot_status(*robot, alias_groups);
  const VectorX<double> q = VectorX<double>::Zero(robot->get_num_positions());
  const VectorX<double> v = VectorX<double>::Zero(robot->get_num_velocities());

  robot_status.Update(0 /* time */, q, v,
                      VectorX<double>::Zero(robot->get_num_actuators()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());
  systems::ConstantValueSource<double>* state_source =
      builder.AddSystem<systems::ConstantValueSource<double>>(
          systems::AbstractValue::Make<HumanoidStatus>(robot_status));

  // Makes a source for qp input.
  QpInput input = paramset.MakeQpInput({}, /* contacts */
                                       {}, /* tracked bodies*/
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

  systems::ConstantValueSource<double>* qp_input_source =
      builder.AddSystem<systems::ConstantValueSource<double>>(
          systems::AbstractValue::Make<QpInput>(input));

  // Connects the diagram.
  builder.Connect(state_source->get_output_port(0),
                  controller->get_input_port_humanoid_status());

  builder.Connect(qp_input_source->get_output_port(0),
                  controller->get_input_port_qp_input());

  builder.ExportOutput(controller->get_output_port_qp_output());

  auto diagram = builder.Build();

  // Uses the simulator to avoid various allocations by hand.
  systems::Simulator<double> sim(*diagram);
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput(sim.get_context());
  sim.Initialize();
  sim.StepTo(controller->get_control_dt());

  // Gets the output from the plan eval block.
  diagram->CalcOutput(sim.get_context(), output.get());
  const QpOutput& qp_output = output->get_data(0)->GetValue<QpOutput>();

  // Expects the solved acceleration to match the desired acceleration well.
  EXPECT_TRUE(drake::CompareMatrices(input.desired_dof_motions().values(),
                                     qp_output.vd(), 1e-9,
                                     drake::MatrixCompareType::absolute));

  // Without any external forces or hitting any contraints, torque = M * vd_d +
  // h.
  VectorX<double> expected_torque =
      robot_status.M() * input.desired_dof_motions().values() +
      robot_status.bias_term();
  EXPECT_TRUE(drake::CompareMatrices(expected_torque, qp_output.dof_torques(),
                                     1e-9, drake::MatrixCompareType::absolute));
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
