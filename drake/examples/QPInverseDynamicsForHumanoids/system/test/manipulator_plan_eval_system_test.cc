#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/manipulator_plan_eval_system.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

GTEST_TEST(testManipPlanEval, ManipPlanEval) {
  const std::string kModelPath =
      drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";

  const std::string kAliasGroupsPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/system/test/config/"
      "iiwa.alias_groups";

  const std::string kControllConfigPath =
      drake::GetDrakePath() +
      "/examples/QPInverseDynamicsForHumanoids/system/test/config/"
      "iiwa.id_controller_config";

  auto robot = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      kModelPath, multibody::joints::kFixed, robot.get());

  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
  alias_groups.LoadFromFile(kAliasGroupsPath);

  VectorX<double> q(robot->get_num_positions());
  VectorX<double> v(robot->get_num_velocities());
  VectorX<double> x_d(q.size() + v.size());
  VectorX<double> vd_d(v.size());

  // Sets estimated position and velocity.
  for (int i = 0; i < q.size(); ++i) q[i] = i;
  for (int i = 0; i < v.size(); ++i) v[i] = 0.1 * i;

  // Sets desired position velocity and acceleration.
  for (int i = 0; i < x_d.size(); ++i) {
    x_d[i] = (-M_PI + i) / 2.;
  }
  for (int i = 0; i < vd_d.size(); ++i) {
    vd_d[i] = -1 + i;
  }

  systems::DiagramBuilder<double> builder;

  // Makes a plan eval block.
  ManipulatorPlanEvalSystem* plan_eval =
      builder.AddSystem<ManipulatorPlanEvalSystem>(*robot, kAliasGroupsPath,
                                                   kControllConfigPath, 0.02);

  // Makes a source for humanoid status that captures estimated state of the
  // robot.
  HumanoidStatus robot_status(*robot, alias_groups);
  robot_status.Update(0, q, v,
                      VectorX<double>::Zero(robot->get_num_actuators()),
                      Vector6<double>::Zero(), Vector6<double>::Zero());
  systems::ConstantValueSource<double>* rs_source =
      builder.AddSystem<systems::ConstantValueSource<double>>(
          systems::AbstractValue::Make<HumanoidStatus>(robot_status));

  // Makes a source for desired position and velocity.
  systems::ConstantVectorSource<double>* state_desired =
      builder.AddSystem<systems::ConstantVectorSource<double>>(x_d);

  // Makes a source for desired acceleration.
  systems::ConstantVectorSource<double>* acc_desired =
      builder.AddSystem<systems::ConstantVectorSource<double>>(vd_d);

  builder.Connect(rs_source->get_output_port(0),
                  plan_eval->get_input_port_humanoid_status());

  builder.Connect(state_desired->get_output_port(),
                  plan_eval->get_input_port_desired_state());

  builder.Connect(acc_desired->get_output_port(),
                  plan_eval->get_input_port_desired_acceleration());

  builder.ExportOutput(plan_eval->get_output_port_qp_input());

  auto diagram = builder.Build();

  // Uses the simulator because I don't have to allocate various things myself.
  systems::Simulator<double> sim(*diagram);
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput(sim.get_context());
  sim.Initialize();
  plan_eval->Initialize(diagram->GetMutableSubsystemContext(
      sim.get_mutable_context(), plan_eval));
  sim.StepTo(plan_eval->get_control_dt());

  // Gets the output from the plan eval block.
  diagram->CalcOutput(sim.get_context(), output.get());
  const QpInput& qp_input = output->get_data(0)->GetValue<QpInput>();

  // Computes the expected output.
  VectorX<double> kp(q.size());
  VectorX<double> kd(v.size());
  plan_eval->get_paramset().LookupDesiredDofMotionGains(&kp, &kd);
  for (int i = 0; i < v.size(); ++i) {
    double expected =
        kp[i] * (x_d[i] - q[i]) + kd[i] * (x_d[i + q.size()] - v[i]) + vd_d[i];
    EXPECT_EQ(expected, qp_input.desired_dof_motions().value(i));
  }
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
